#!/usr/bin/env python3
"""Small text drive log for road tests without a laptop.

Runs while onroad (started by manager) and writes /data/drivelog/<date>-<time>.txt:
  * one status line per second (speed, set speed, engage state, pedals, steering, lead,
    average CPU %, max CPU temp, memory %, free storage GB)
  * an event line right away when engage state or the alert text changes, a fault event appears or clears
    (canError, steerUnavailable, commIssue, radarFault, ...), controlsd lags, or a process stops
  * python process crashes write their traceback to crash-<time>-<name>.txt (selfdrive/manager/process.py)

Only the newest message of each service is read (conflated sockets, 10 checks per second),
so the CPU cost stays small. The newest KEEP_FILES files are kept.

Read later over SSH:  ls /data/drivelog  /  cat /data/drivelog/<file>
"""
import datetime
import os
import time

import cereal.messaging as messaging

LOG_DIR = "/data/drivelog"
KEEP_FILES = 30
SERVICES = ("carState", "controlsState", "carControl", "radarState", "deviceState", "managerState", "carEvents",
            "gpsLocationExternal", "pandaStates")

# events that happen in normal driving (buttons, pedals, lane change prompts): not written as faults
NORMAL_EVENTS = {
  "buttonCancel", "buttonEnable", "pcmEnable", "pcmDisable", "pedalPressed", "gasPressedOverride", "steerOverride",
  "preEnableStandstill", "preLaneChangeLeft", "preLaneChangeRight", "laneChange", "resumeRequired",
  "cruisePaused", "cruiseResume", "trafficSignGreen", "trafficSignChanged", "autoHold", "trafficStopping",
  "speedDown", "audioPrompt", "audioRefuse", "stopStop", "audioLaneChange", "audioTurn", "brakeHold",
}
LAG_EVENT_MS = 100  # controlsd frame late by more than this -> event (at most every 10 s)

HEADER = ("# time | v km/h | set | en act lat lon | gas brk str | angle torque | steerCmd accel | "
          "lead d/vrel | cpu% temp mem% freeGB | lag ms | gps fix sats strong cno accm | fdrop% radarlag | "
          "carSCC main on mode | cluster info alert fail | lane L R cam camWarn | brakeLight | btn/s RESoff RESon SET GAP CANCEL | "
          "FCA fail status camFCW fusion | leadSrc")

# carControl.debugTextCC from the Hyundai carcontroller (drive_record_text): SCC radar -> cluster codes, the stock
# camera's lane state and the cruise buttons openpilot sent (counts since start)
OPT_KEYS = ("i", "a", "f", "m", "cl", "cw", "r0", "r1", "s", "g", "c")
BTN_KEYS = ("r0", "r1", "s", "g", "c")
# added 2026-11-23 (-1 when the car / an older carcontroller does not give them): the forward collision-avoidance
# fault (FCA_Failinfo / AEB_Failinfo, non-zero = the cluster's '전방 충돌방지 보조 시스템을 점검하세요') and state,
# and the stock camera's FCW state and radar fusion state from LKAS11
FCA_KEYS = ("fi", "fs", "cf", "cu")
PANDA_ERR_EVENT_S = 10.  # at most one 'panda CAN errors' event this often
# In the car the clock often starts at 1970-01-01 09:00: no network time yet and no panda RTC. From this year on the
# clock (or a GPS time) is taken as right.
CLOCK_OK_YEAR = 2024


def clock_ok(t=None):
  return datetime.datetime.fromtimestamp(time.time() if t is None else t).year >= CLOCK_OK_YEAR


def parse_opt(text):
  if not text.startswith("OPT "):
    return None
  try:
    d = dict(kv.split("=") for kv in text[4:].split())
    opt = {k: int(d[k]) for k in OPT_KEYS}
    opt.update({k: int(d.get(k, -1)) for k in FCA_KEYS})
    return opt
  except (ValueError, KeyError):
    return None


DIAG_FILE = os.path.join(LOG_DIR, "diag.txt")
DIAG_MAX_BYTES = 2_000_000


def diag_write(text):
  """Append one diagnostic line (controlsd: which services made commIssue / radarFault). Only called on a change,
  never raises, and starts over when the file passes DIAG_MAX_BYTES."""
  try:
    os.makedirs(LOG_DIR, exist_ok=True)
    mode = "w" if os.path.isfile(DIAG_FILE) and os.path.getsize(DIAG_FILE) > DIAG_MAX_BYTES else "a"
    with open(DIAG_FILE, mode, encoding="utf8") as f:
      f.write(f"{datetime.datetime.now():%m-%d %H:%M:%S.%f}"[:-3] + f" {text}\n")
  except Exception:  # noqa: BLE001
    pass


def b(x):
  return "1" if x else "0"


class DriveLog:
  def __init__(self, out):
    self.out = out
    self.socks = {s: messaging.sub_sock(s, conflate=True) for s in SERVICES}
    self.last = {}
    self.prev_engage = None
    self.prev_alert = ""
    self.dead = set()
    self.events = set()
    self.next_line = time.monotonic()
    self.next_lag_event = 0.
    # GPS (uno uBlox or phone, see GpsSource): satellites from ubloxGnss.measurementReport, which shares the
    # service with other uBlox messages, so it is drained instead of conflated
    self.gnss_sock = messaging.sub_sock("ubloxGnss")
    self.gps_sats = self.gps_strong = self.gps_cno = 0
    self.gnss_t = self.loc_t = -100.
    self.prev_fix = None
    self.clock_tried = False  # gps_clock: done (or not needed) for this drive
    self.gps_ant = None
    # read once a second (not in SERVICES: modelV2 is big, procLog only comes every 2 s)
    self.model_sock = messaging.sub_sock("modelV2", conflate=True)
    self.proc_sock = messaging.sub_sock("procLog", conflate=True)
    # navigation link (road_speed_limiter): written when the link, the turn or the camera changes (navi_events)
    self.navi_sock = messaging.sub_sock("roadLimitSpeed", conflate=True)
    self.prev_navi = None
    self.next_navi_event = 0.
    self.proc_prev = None  # (time, {pid: (name, cpu seconds)})
    self.next_cpu_event = time.monotonic() + 30.
    # car SCC / cluster / buttons (parse_opt)
    self.prev_car_scc = None
    self.prev_disp = None
    self.prev_cw = None
    self.prev_btn = None       # button counts at the last check
    self.line_btn = None       # button counts at the last status line
    self.res_off_n = 0         # RES sent while the car's SCC was off, in the current run
    self.res_off_last = None   # time of the last such RES
    self.prev_fca = None
    # panda: safety mode (the moment openpilot takes the car's messages over), faults, CAN errors
    self.prev_panda = None
    self.prev_panda_errs = None
    self.next_panda_err_event = 0.
    # '주행 중 밝기' (screen_events)
    self.drive_dim = 0
    self.next_dim_read = 0.
    self.prev_screen = None

  def context(self, cs, cc, ctl):
    """Short state for an event line: what the driver and openpilot were doing at that moment."""
    hud = cc.hudControl
    rs = self.last.get("radarState")
    lead = f"{rs.leadOne.dRel:.0f}m" if rs is not None and rs.leadOne.status else "-"
    vset = f"{ctl.vCruise:.0f}" if ctl is not None else "-"
    return (f"[v {cs.vEgo * 3.6:.0f} set {vset} oplong {b(cc.longActive)} brk {b(cs.brakePressed)} "
            f"brkLight {b(cs.brakeLights)} gas {b(cs.gasPressed)} lane {b(hud.leftLaneVisible)}{b(hud.rightLaneVisible)} "
            f"lead {lead} angle {cs.steeringAngleDeg:.0f}]")

  def scc_events(self, cs, cc, ctl, opt, now):
    ctx = self.context(cs, cc, ctl)
    car_scc = (cs.cruiseState.available, cs.cruiseState.enabled)
    if self.prev_car_scc is not None and car_scc != self.prev_car_scc:
      what = "on" if car_scc[1] else ("off" if car_scc[0] else "main off")
      self.event(f"car SCC {what} (mode {opt['m']}, cluster info {opt['i']}) {ctx}")
    self.prev_car_scc = car_scc

    disp = (opt["i"], opt["a"], opt["f"])
    if self.prev_disp is not None and disp != self.prev_disp:
      p = self.prev_disp
      self.event(f"cluster SCC info {p[0]}->{disp[0]} alert {p[1]}->{disp[1]} fail {p[2]}->{disp[2]} {ctx}")
    self.prev_disp = disp

    fca = tuple(opt[k] for k in FCA_KEYS)
    if self.prev_fca is not None and fca != self.prev_fca:
      p = self.prev_fca
      self.event(f"car FCA fail {p[0]}->{fca[0]} status {p[1]}->{fca[1]} (camera FCW {p[2]}->{fca[2]} "
                 f"fusion {p[3]}->{fca[3]}) {ctx}")
    self.prev_fca = fca

    if self.prev_cw is not None and opt["cw"] != self.prev_cw:
      self.event(f"stock camera warning {self.prev_cw}->{opt['cw']} (camera lane state {opt['cl']}) {ctx}")
    self.prev_cw = opt["cw"]

    btn = [opt[k] for k in BTN_KEYS]
    if self.prev_btn is not None and all(n >= o for n, o in zip(btn, self.prev_btn)):
      d = [n - o for n, o in zip(btn, self.prev_btn)]
      if d[0] > 0:  # RES while the car's SCC is off: the car may answer with 'SCC 작동조건이 아님'
        if self.res_off_last is None:
          self.event(f"RES sent while car SCC off: start {ctx}")
          self.res_off_n = 0
        self.res_off_n += d[0]
        self.res_off_last = now
      if d[4] > 0:
        self.event(f"CANCEL sent {ctx}")
    elif self.prev_btn is not None:  # counts went back: controlsd restarted
      self.line_btn = None
    self.prev_btn = btn
    if self.res_off_last is not None and now - self.res_off_last > 2.:
      self.event(f"RES sent while car SCC off: stop ({self.res_off_n} sent, car SCC {'on' if car_scc[1] else 'off'}) {ctx}")
      self.res_off_last = None

  def event(self, text):
    self.out.write(f"{datetime.datetime.now():%H:%M:%S} ** {text}\n")

  def gps_clock(self, loc):
    """Clock still at 1970 (no network time, no panda RTC): set it once from the GPS fix, like upstream openpilot's
    timed. Then the drive file gets its real name (main) and crash files their real time. Only forward from 1970,
    and only when the GPS time itself looks right."""
    if clock_ok():
      self.clock_tried = True
      return
    gps_s = loc.unixTimestampMillis / 1000.
    if not clock_ok(gps_s):
      return  # no GPS time in this message yet: try again on the next one
    self.clock_tried = True
    before = datetime.datetime.now()
    try:
      time.clock_settime(time.CLOCK_REALTIME, gps_s)
      self.event(f"clock set from GPS: {before:%Y-%m-%d %H:%M:%S} -> {datetime.datetime.now():%Y-%m-%d %H:%M:%S}")
    except (OSError, AttributeError) as e:
      self.event(f"clock not set from GPS: {type(e).__name__}: {e}")

  def navi_events(self, now):
    """What the navigation link delivers (Soul EV tester 2026-11-25: connected but no turn arrows). active (see
    road_speed_limiter): 200+ route guidance / Tmap SDI data, 1-99 an NDA app linked (its cameras come in the NDA
    fields). naviSource: the port it came in on (the screen's NAVI CARROT / NAVI NDA badge). A line on every change of
    link, turn or camera, at most every 2 s."""
    msg = messaging.recv_one_or_none(self.navi_sock)
    if msg is None:
      return
    r = msg.roadLimitSpeed
    src = {1: "CARROT", 2: "NDA"}.get(r.naviSource, "-")
    link = ("guidance" if r.active >= 200 else "linked" if r.active > 0 or r.naviSource > 0 else "none") + " via " + src
    turn = r.xTurnInfo if r.xDistToTurn > 0 else -1
    cam = (r.xSpdLimit, r.camType) if r.xSpdLimit > 0 and r.xSpdDist > 0 else None
    key = (link, turn, cam, r.roadLimitSpeed)
    if key != self.prev_navi and now >= self.next_navi_event:
      self.prev_navi = key
      self.next_navi_event = now + 2.
      self.event(f"navi {link} (active {r.active}): turn {turn} in {r.xDistToTurn} m, camera {r.xSpdLimit} km/h in "
                 f"{r.xSpdDist} m (type {r.camType}, sign {r.xSignType}), road limit {r.roadLimitSpeed}, "
                 f"NDA camera {r.camLimitSpeed} in {r.camLimitSpeedLeftDist} m [{r.xRoadName[:40]}]")

  def step(self):
    for s, sock in self.socks.items():
      msg = messaging.recv_one_or_none(sock)
      if msg is not None:
        self.last[s] = getattr(msg, s)

    cs, ctl, cc = self.last.get("carState"), self.last.get("controlsState"), self.last.get("carControl")

    now = time.monotonic()
    for msg in messaging.drain_sock(self.gnss_sock):
      if msg.ubloxGnss.which() == "measurementReport":
        cnos = sorted((m.cno for m in msg.ubloxGnss.measurementReport.measurements if m.cno > 0), reverse=True)
        self.gps_sats, self.gps_strong = len(cnos), sum(c >= 30 for c in cnos)
        self.gps_cno = sum(cnos[:4]) // len(cnos[:4]) if cnos else 0
        self.gnss_t = now
      elif msg.ubloxGnss.which() == "hwStatus":
        hw = msg.ubloxGnss.hwStatus
        ant = (str(hw.aStatus), str(hw.aPower))
        if ant != self.gps_ant:  # uBlox antenna supervisor: ok / short / open / dontknow
          self.event(f"gps antenna {ant[0]} power {ant[1]} (noise {hw.noisePerMS}, agc {hw.agcCnt}, jam {hw.jamInd})")
          self.gps_ant = ant
    loc = self.last.get("gpsLocationExternal")
    if "gpsLocationExternal" in self.last and loc is not self.last.get("_loc_seen"):
      self.last["_loc_seen"] = loc
      self.loc_t = now
    loc_alive = loc is not None and now - self.loc_t < 3.
    fix = loc_alive and bool(loc.flags & 1)
    if self.prev_fix is not None and fix != self.prev_fix:
      self.event(f"gps fix {'on' if fix else 'lost'} (sats {self.gps_sats}, strong {self.gps_strong}"
                 + (f", acc {loc.accuracy:.0f} m)" if fix else ")"))
    self.prev_fix = fix
    if fix and not self.clock_tried:
      self.gps_clock(loc)

    if ctl is not None:
      engage = (ctl.enabled, ctl.active)
      if self.prev_engage is not None and engage != self.prev_engage:
        p = self.prev_engage
        self.event(f"engage {b(p[0])}{b(p[1])} -> {b(engage[0])}{b(engage[1])} ({ctl.alertText1} {ctl.alertText2})")
      self.prev_engage = engage
      alert = f"{ctl.alertText1} {ctl.alertText2}".strip()
      if alert and alert != self.prev_alert:
        self.event(f"alert: {alert}")
      self.prev_alert = alert
      self.screen_events(ctl, now)

    ms = self.last.get("managerState")
    if ms is not None:
      for p in ms.processes:
        down = p.shouldBeRunning and not p.running
        if down and p.name not in self.dead:
          self.event(f"process stopped: {p.name} (exit {p.exitCode})")
          self.dead.add(p.name)
        elif not down and p.name in self.dead:
          self.event(f"process back: {p.name}")
          self.dead.discard(p.name)

    self.panda_events()
    self.navi_events(now)

    ev = self.last.get("carEvents")
    if ev is not None:
      names = {str(e.name) for e in ev} - NORMAL_EVENTS
      added, cleared = names - self.events, self.events - names
      if added or cleared:
        self.event("events " + " ".join([f"+{n}" for n in sorted(added)] + [f"-{n}" for n in sorted(cleared)]))
      self.events = names

    now = time.monotonic()
    opt = parse_opt(cc.debugTextCC) if cc is not None else None
    if opt is not None and cs is not None:
      self.scc_events(cs, cc, ctl, opt, now)

    if ctl is not None and ctl.cumLagMs > LAG_EVENT_MS and now >= self.next_lag_event:
      self.event(f"controlsd lag {ctl.cumLagMs:.0f} ms")
      self.next_lag_event = now + 10.

    now = time.monotonic()
    if now < self.next_line or cs is None:
      return
    self.next_line = now + 1.0

    parts = [f"{datetime.datetime.now():%H:%M:%S}", f"{cs.vEgo * 3.6:5.1f}"]
    parts.append(f"{ctl.vCruise:3.0f}" if ctl is not None else "-")
    if ctl is not None and cc is not None:
      parts.append(f"{b(ctl.enabled)} {b(ctl.active)} {b(cc.latActive)} {b(cc.longActive)}")
    else:
      parts.append("- - - -")
    parts.append(f"{b(cs.gasPressed)} {b(cs.brakePressed)} {b(cs.steeringPressed)}")
    parts.append(f"{cs.steeringAngleDeg:6.1f} {cs.steeringTorque:6.0f}")
    parts.append(f"{cc.actuators.steer:5.2f} {cc.actuators.accel:5.2f}" if cc is not None else "- -")
    rs = self.last.get("radarState")
    if rs is not None and rs.leadOne.status:
      parts.append(f"{rs.leadOne.dRel:5.1f} {rs.leadOne.vRel * 3.6:5.1f}")
    else:
      parts.append("- -")
    ds = self.last.get("deviceState")
    # optech (2026-11-26): free storage measured here, so it is in the log even when thermald (deviceState) is down
    try:
      st = os.statvfs("/data")
      free_gb = f"{st.f_bavail * st.f_frsize / 1e9:5.1f}"
    except OSError:
      free_gb = "-"
    if ds is not None:
      cpu = sum(ds.cpuUsagePercent) // len(ds.cpuUsagePercent) if len(ds.cpuUsagePercent) else 0
      temp = max(ds.cpuTempC) if len(ds.cpuTempC) else 0
      parts.append(f"{cpu:3d} {temp:4.0f} {ds.memoryUsagePercent:3d} {free_gb}")
    else:
      parts.append(f"- - - {free_gb}")
    parts.append(f"{ctl.cumLagMs:5.0f}" if ctl is not None else "-")
    if now - self.gnss_t < 3. or loc_alive:
      acc = f"{loc.accuracy:5.0f}" if fix else "    -"
      parts.append(f"{b(fix)} {self.gps_sats:2d} {self.gps_strong:2d} {self.gps_cno:2d} {acc}")
    else:
      parts.append("- - - - -")
    md = messaging.recv_one_or_none(self.model_sock)
    rs_lag = f"{rs.cumLagMs:5.0f}" if rs is not None else "-"
    parts.append((f"{md.modelV2.frameDropPerc:4.1f} " if md is not None else "- ") + rs_lag)
    parts.append(f"{b(cs.cruiseState.available)} {b(cs.cruiseState.enabled)} {opt['m']}" if opt is not None else
                 f"{b(cs.cruiseState.available)} {b(cs.cruiseState.enabled)} -")
    if opt is not None:
      hud = cc.hudControl
      parts.append(f"{opt['i']} {opt['a']} {opt['f']}")
      parts.append(f"{b(hud.leftLaneVisible)} {b(hud.rightLaneVisible)} {opt['cl']} {opt['cw']}")
    else:
      parts += ["- - -", "- - - -"]
    parts.append(b(cs.brakeLights))
    btn = [opt[k] for k in BTN_KEYS] if opt is not None else None
    if btn is not None and self.line_btn is not None:
      parts.append(" ".join(str(n - o) for n, o in zip(btn, self.line_btn)))
    else:
      parts.append("- - - - -")
    self.line_btn = btn
    parts.append(" ".join(str(opt[k]) for k in FCA_KEYS) if opt is not None else "- - - -")
    # where leadOne came from: R = a radar point (camera-matched or the low-speed rule), V = camera only (added 2026-11-24)
    parts.append(("R" if rs.leadOne.radar else "V") if rs is not None and rs.leadOne.status else "-")
    self.out.write(" | ".join(parts) + "\n")
    self.cpu_top(now)

  def screen_events(self, ctl, now):
    """'주행 중 밝기' (OptechDriveDim, ui.cc Device::updateBrightness) set: which warning brings a dimmed screen back.
    Only a warning drawn on the screen does (status not normal, size not none); a prompt without a box only changes
    the status (Soul EV tester 2026-11-27: 0.5 s flashes at '화면 밝기 0%'). Touches are not visible here."""
    if now >= self.next_dim_read:
      self.next_dim_read = now + 10.
      try:
        from common.params import Params
        self.drive_dim = int(Params().get("OptechDriveDim", encoding="utf8") or 0)
      except Exception:  # noqa: BLE001
        self.drive_dim = 0
    reason = None
    if self.drive_dim != 0 and str(ctl.alertStatus) != "normal":
      text = f"{ctl.alertText1} {ctl.alertText2}".strip()
      if str(ctl.alertSize) != "none":
        reason = f"screen bright: {ctl.alertStatus} warning '{text}'"
      else:
        reason = f"screen stays dark: {ctl.alertStatus} prompt without a box '{text}'"
    if reason != self.prev_screen:
      if reason is not None:
        self.event(reason)
      elif self.prev_screen.startswith("screen bright"):
        self.event("screen: warning over, dimming rules again")
      self.prev_screen = reason

  def panda_events(self):
    """Panda safety mode / controls allowed / faults / harness changes right away (the switch to the car's safety mode
    is when openpilot starts replacing the car's messages), and CAN send / forward / receive errors when they grow."""
    ps = self.last.get("pandaStates")
    if ps is None or len(ps) == 0:
      return
    p = ps[0]
    state = (str(p.safetyModel), p.safetyParam, b(p.controlsAllowed), str(p.faultStatus),
             ",".join(str(f) for f in p.faults), str(p.harnessStatus), b(p.heartbeatLost), b(p.ignitionLine))
    if state != self.prev_panda:
      self.event("panda safety {} ({}) allowed {} fault {} [{}] harness {} heartbeatLost {} ign {}".format(*state)
                 + (f" (was {self.prev_panda[0]})" if self.prev_panda is not None else ""))
    self.prev_panda = state
    errs = (p.canSendErrs, p.canFwdErrs, p.canRxErrs)
    now = time.monotonic()
    if self.prev_panda_errs is None:
      self.prev_panda_errs = errs
    elif now >= self.next_panda_err_event and errs != self.prev_panda_errs:
      d = [n - o for n, o in zip(errs, self.prev_panda_errs)]
      self.event(f"panda CAN errors send +{d[0]} fwd +{d[1]} rx +{d[2]} (total {errs[0]} {errs[1]} {errs[2]})")
      self.next_panda_err_event = now + PANDA_ERR_EVENT_S
      self.prev_panda_errs = errs

  def cpu_top(self, now):
    """Every 60 s: the processes using the most CPU since the last snapshot (percent of one core)."""
    pl = messaging.recv_one_or_none(self.proc_sock)
    if pl is None:
      return
    snap = {p.pid: (p.name, p.cpuUser + p.cpuSystem) for p in pl.procLog.procs}
    if self.proc_prev is None:
      self.proc_prev = (now, snap)
      return
    if now >= self.next_cpu_event:
      dt = now - self.proc_prev[0]
      old = self.proc_prev[1]
      use = [(name, (t - old[pid][1]) / dt * 100.) for pid, (name, t) in snap.items() if pid in old and dt > 0]
      top = sorted(use, key=lambda x: -x[1])[:6]
      self.event("cpu top: " + ", ".join(f"{n} {u:.0f}%" for n, u in top))
      self.next_cpu_event = now + 60.
      self.proc_prev = (now, snap)


def free_path(start):
  """<date>-<time>.txt for a drive that started at `start`, -1, -2 ... when that name is taken: with the clock at
  1970 the same name comes back, and a drive is never appended to an older one."""
  base = os.path.join(LOG_DIR, start.strftime("%Y%m%d-%H%M%S"))
  path, n = base + ".txt", 1
  while os.path.exists(path):
    path, n = f"{base}-{n}.txt", n + 1
  return path


def rename_for_clock(log, path, start_mono):
  """The drive started with the clock at 1970 and now it is set (GPS or network time): give the file its real start
  time, so the phone page and the zip show the right date."""
  start = datetime.datetime.now() - datetime.timedelta(seconds=time.monotonic() - start_mono)
  new = free_path(start)
  try:
    os.rename(path, new)  # the open file keeps being written under the new name
  except OSError as e:
    log.event(f"drive file not renamed: {e}")
    return path
  log.event(f"clock set: this drive started {start:%Y-%m-%d %H:%M:%S}, the times above are the unset clock "
            f"(file {os.path.basename(path)} -> {os.path.basename(new)})")
  return new


def main():
  os.makedirs(LOG_DIR, exist_ok=True)
  # drive logs (<date>-<time>[-n].txt, also 1970... when the clock was not set yet) and crash tracebacks are kept
  # separately, newest KEEP_FILES of each by modification time
  names = [f for f in os.listdir(LOG_DIR) if f.endswith(".txt")]
  groups = ([f for f in names if f[:1].isdigit()], [f for f in names if f.startswith("crash-")])
  for group in groups:
    for f in sorted(group, key=lambda x: os.path.getmtime(os.path.join(LOG_DIR, x)))[:-KEEP_FILES]:
      try:
        os.remove(os.path.join(LOG_DIR, f))
      except OSError:
        pass

  start_mono = time.monotonic()
  path = free_path(datetime.datetime.now())
  name_unset = not clock_ok()  # named 1970...: renamed once the clock is set (DriveLog.gps_clock or network time)
  with open(path, "a", buffering=1, encoding="utf8") as out:
    out.write(HEADER + "\n")
    log = DriveLog(out)
    try:
      from common.params import Params
      log.event("gps source: " + ("phone" if Params().get("GpsSource", encoding="utf8") == "1" else "panda/uno"))
    except Exception:  # noqa: BLE001
      pass
    errors = 0
    next_sync = time.monotonic() + 5.
    while True:
      try:
        log.step()
      except Exception as e:  # noqa: BLE001  (one bad message must not stop the log)
        errors += 1
        if errors <= 5:
          log.event(f"drivelog error: {type(e).__name__}: {e}")
      if time.monotonic() >= next_sync:  # a power cut at ignition off otherwise leaves the last minutes as zeros
        next_sync = time.monotonic() + 5.
        try:
          os.fsync(out.fileno())
        except OSError:
          pass
        if name_unset and clock_ok():
          name_unset = False
          path = rename_for_clock(log, path, start_mono)
      time.sleep(0.1)


if __name__ == "__main__":
  main()
