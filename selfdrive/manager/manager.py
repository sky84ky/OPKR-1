#!/usr/bin/env python3
import datetime
import os
import signal
import subprocess
import sys
import traceback
from multiprocessing import Process
from typing import List, Tuple, Union

import cereal.messaging as messaging
import selfdrive.sentry as sentry
from common.basedir import BASEDIR
from common.params import Params, ParamKeyType
from common.text_window import TextWindow
from selfdrive.boardd.set_time import set_time
from selfdrive.hardware import HARDWARE, PC, EON
from selfdrive.manager.helpers import unblock_stdout
from selfdrive.manager.process import ensure_running, launcher
from selfdrive.manager.process_config import managed_processes
from selfdrive.athena.registration import register, UNREGISTERED_DONGLE_ID
from selfdrive.swaglog import cloudlog, add_file_handler
from selfdrive.version import is_dirty, get_commit, get_version, get_origin, get_short_branch, \
                              terms_version, training_version
from selfdrive.hardware.eon.apk import system

sys.path.append(os.path.join(BASEDIR, "pyextra"))


def optech_navi_link_migrate(params) -> None:
  """optech 2026-11-25: 내비 연동 방식 (OptechNaviLinkMode) names the phone app now: 0 사용 안 함, 1 TmapNda, 2 당근내비
  (it was 0 / 1 APM / 2 APM + 순정 내비 / 3 NDA). AutoNaviSpeedCtrl stays the internal switch road_speed_limiter and
  cruise_helper read: 0 camera slowdown off, 1 no app (built-in DB only), 3 NDA + carrot protocol + the car's navi
  (TmapNda speaks both), 2 carrot protocol + the car's navi (당근내비). APM (APilotMan + Mappy) is no longer received.
  Old values move once: 3 NDA -> 1, 1 APM (AutoNaviSpeedCtrl 1) -> 2; new values are left as they are, so running
  this at every boot changes nothing after the first time. (Old 1 with the slowdown off can't be told from the new 1
  and stays 1: TmapNda's mode takes carrot apps too.)"""
  try:
    mode = int(params.get("OptechNaviLinkMode") or b"-1")
  except ValueError:
    mode = -1
  try:
    ctrl = int(params.get("AutoNaviSpeedCtrl") or b"0")
  except ValueError:
    ctrl = 0
  if mode == 3:
    mode = 1
  elif mode == 1 and ctrl == 1:
    mode = 2
  elif not 0 <= mode <= 2:
    mode = 2 if ctrl == 2 else 1
  params.put("OptechNaviLinkMode", str(mode))
  if ctrl > 0:
    params.put("AutoNaviSpeedCtrl", str({0: 1, 1: 3, 2: 2}[mode]))


def manager_init() -> None:
  # update system time from panda
  set_time(cloudlog)

  # save boot log
  #subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))

  params = Params()

  # optech: panda relay behaviour for the selected car (boardd sends it with NO_OUTPUT); car_helpers keeps it current
  try:
    selected = params.get("SelectedCar", encoding="utf8")
    if selected:
      from selfdrive.car.optech_relay import set_idle_passthrough
      set_idle_passthrough(selected, params)
  except Exception:
    cloudlog.exception("optech: PandaIdlePassthrough from SelectedCar failed")
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)

  default_params: List[Tuple[str, Union[str, bytes]]] = [
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("GsmMetered", "1"),
    ("HasAcceptedTerms", "0"),
    ("LanguageSetting", "main_ko"),
    ("OpenpilotEnabledToggle", "1"),
    ("ShowDebugUI", "0"),
    ("OPKRServer", "0"),
    ("ShowDateTime", "0"),
    ("ShowHudMode", "7"),
    ("ShowSteerRotate", "1"),
    ("ShowPathEnd", "1"),
    ("ShowAccelRpm", "1"),
    ("ShowTpms", "0"),
    ("ShowSteerMode", "2"),
    ("ShowDeviceState", "1"),
    ("ShowGpsInfo", "1"),
    ("AutoShutdown", "1"),
    ("GpsSource", "1"),
    # ("EngageWithLong" '사용 방식' removed 2026-11-27: carrot's cruise button rule, nothing reads it any more)
    # Ray EV / Porter II EV (no SCC): each car its own settings page (start tab, shown when the car is chosen)
    ("RaySpeedButtons", "0"),      # let openpilot press the cruise buttons to set the speed (no pedal)
    ("RayPedal", "0"),             # comma pedal: openpilot long through the accelerator
    ("RayPedalMax", "70"),         # pedal command cap, % (StarPilot Ray calibration allows 70)
    ("RayPedalStrong", "0"),       # carrot PR 570: twice the pedal for the same command (2024 Ray EV owner's fix)
    ("RayPedalKi", "50"),          # speed error integral gain x0.001 with the pedal (carrot's Ray candidate 50)
    ("PorterSpeedButtons", "0"),
    ("PorterPedal", "0"),
    ("PorterPedalMax", "40"),      # no Porter calibration yet: lower cap
    # Chevrolet Bolt EV 2017-21 without ACC (ds5ajd/openpilot 0814)
    ("BoltPedal", "0"),            # comma pedal: openpilot long through the accelerator
    ("BoltPedalMax", "100"),       # pedal command cap, % on top of StarPilot's own speed cap (100 = StarPilot as is)
    ("BoltYear2017", "0"),         # 2017 Bolt EV: StarPilot's 2017 steering tune and limits
    ("ShowConnInfo", "1"),
    ("ShowLaneInfo", "1"),
    ("ShowBlindSpot", "1"),
    ("ShowGapInfo", "1"),
    ("ShowDmInfo", "1"),
    ("ShowRadarInfo", "1"),
    ("MixRadarInfo", "0"),
    ("ShowZOffset", "122"),
    ("ShowPathMode", "9"),
    ("ShowPathColor", "12"),
    ("ShowPathModeCruiseOff", "0"),
    ("ShowPathColorCruiseOff", "19"),
    ("ShowPathModeLane", "12"),
    ("ShowPathColorLane", "13"),
    ("ShowPathWidth", "100"),
    ("ShowPlotMode", "0"),
    ("AutoResumeFromGas", "1"),
    ("AutoResumeFromGasSpeed", "30"),
    ("AutoResumeFromGasSpeedMode", "0"),    
    ("AutoCancelFromGasMode", "1"),    
    ("OpkrPrebuiltOn", "1"),
    ("OpkrHotspotOnBoot", "0"),
    ("OPKRTimeZone", "Asia/Seoul"),    
    ("AutoCurveSpeedCtrlUse", "2"),
    ("CarrotTurnAccel", "0"),
    ("CarrotRadar", "0"),
    ("CarrotCutInPredecel", "0"),
    ("CarrotCutOutRelief", "0"),
    ("CarrotStoppingLead", "0"),
    ("SpeedCamDB", "0"),
    ("AutoCurveSpeedFactor", "100"),
    ("AutoCurveSpeedFactorIn", "10"),
    ("AutoTurnControl", "0"),
    ("AutoTurnControlSpeedLaneChange", "60"),
    ("AutoTurnControlSpeedTurn", "20"),
    ("AutoTurnControlTurnEnd", "3"),
    ("AutoLaneChangeSpeed", "30"),
    ("AutoNaviSpeedCtrl", "0"),
    ("AutoNaviSpeedCtrlMode", "0"),
    ("AutoNaviSpeedCtrlStart", "22"),
    ("AutoNaviSpeedCtrlEnd", "6"),
    ("AutoNaviSpeedBumpDist", "10"),
    ("AutoNaviSpeedBumpSpeed", "35"),
    ("AutoNaviSpeedSafetyFactor", "100"),
    ("OptechBrakeResumeMode", "1"),
    ("OptechLdwsCar", "0"),       # 토글 > LDWS 차량 (OPKR LdwsCarFix: LKAS11 CF_Lkas_LdwsOpt_USM = 3)
    ("OptechSignalEnable", "1"),  # 주행 > 신호등 > 신호등 사용 (HUD traffic light + green light alert)
    ("OptechGreenAlert", "1"),   # 주행 > 신호등 > 초록불 알림 (StarPilot green light alert)
    ("OptechGasResume", "0"),    # 주행 > 브레이크 > 악셀로 다시 켜기 (off: the driver turns it on)
    ("OptechOffShutdown", "0"),  # 고급 > 시동 끈 뒤 꺼짐 (seconds, 0 off)
    ("OptechSettingsTimeout", "180"),  # 화면 > 화면 유지 시간 (seconds, -1 항상)
    ("OptechDriveDim", "0"),  # 화면 > 주행 중 밝기 (%, 0 off)
    ("OptechAutoE2E", "0"),  # 고급 > 자동 E2E / ACC 전환 (longitudinal_planner.update_auto_e2e)
    ("OptechShowRecordBtn", "0"),  # 화면 > 녹화 버튼 (0 hidden)
    ("OptechShowGpsSpeed", "1"),  # 화면 > GPS 속도 (설정 속도 칸 오른쪽, 1 shown)
    ("OptechLatMethod", "0"),
    ("OptechPidKp", "25"),
    ("OptechPidKi", "40"),
    ("OptechPidKf", "7"),
    ("OptechIndiInner", "35"),
    ("OptechIndiOuter", "20"),
    ("OptechIndiTime", "14"),
    ("OptechIndiAct", "20"),
    ("OptechLqrScale", "1500"),
    ("OptechLqrKi", "16"),
    ("OptechLqrDcGain", "265"),  # 조향 > 조향 방식 (0 토크, 1 PID, 2 INDI, 3 LQR) and their values (OPKR defaults)
    ("AutoNaviSpeedDecelRate", "80"),
    ("AutoRoadLimitCtrl", "0"),
    ("AutoResumeFromBrakeRelease", "1"),
    ("AutoResumeFromBrakeReleaseDist", "20"),
    ("AutoResumeFromBrakeReleaseLeadCar", "1"),
    ("AutoResumeFromBrakeCarSpeed", "40"),
    ("AutoResumeFromBrakeReleaseTrafficSign", "1"),
    ("LongControlActiveSound", "1"),
    ("StartAccelApply", "0"),
    ("StopAccelApply", "30"),
    ("TrafficStopDistanceAdjust", "400"),
    ("AutoSpeedUptoRoadSpeedLimit", "100"),
    ("ApplyLongDynamicCost", "0"), 
    ("AutoSpeedAdjustWithLeadCar", "0"),   
    ("TrafficStopAccel", "80"),     
    ("TrafficStopModelSpeed", "0"),         
    ("TrafficStopMode", "0"),         
    ("CruiseButtonMode", "0"),      
    ("CruiseSpeedUnit", "10"),      
    ("InitMyDrivingMode", "3"),      
    ("MySafeModeFactor", "80"),      
    ("LiveSteerRatioApply", "100"),      
    ("MyEcoModeFactor", "80"),  
    ("CruiseMaxVals1", "160"),
    ("CruiseMaxVals2", "120"),
    ("CruiseMaxVals3", "100"),
    ("CruiseMaxVals4", "80"),
    ("CruiseMaxVals5", "70"),
    ("CruiseMaxVals6", "60"),
    ("PrevCruiseGap", "4"),      
    ("CruiseSpeedMin", "10"),
    ("AutoSyncCruiseSpeedMax", "120"),       
    ("StopDistance", "600"), 
    ("CustomMapbox", "0"),    
    ("LongitudinalTuningKpV", "100"),     
    ("LongitudinalTuningKiV", "200"),     
    ("LongitudinalTuningKf", "100"),     
    ("LongitudinalActuatorDelayUpperBound", "50"),     
    ("LongitudinalActuatorDelayLowerBound", "50"),     
    ("EnableRadarTracks", "0"),      
    ("EnableAutoEngage", "0"),      
    ("SccConnectedBus2", "0"),   
    ("TFollowSpeedRatio", "110"),
    ("TFollowGap1", "110"),
    ("TFollowGap2", "120"),
    ("TFollowGap3", "140"),
    ("TFollowGap4", "160"),
    ("JerkStartLimit", "10"),    
    ("KeepEngage", "1"),
    ("UseLaneLineSpeed", "0"),    
    ("PathOffset", "0"),  
    ("PathCostApply", "100"),
    ("HapticFeedbackWhenSpeedCamera", "0"),       
    ("MaxAngleFrames", "89"),       
    ("SoftHoldMode", "0"),       
    ("ApplyModelDistOrder", "30"),       
    ("TrafficStopAdjustRatio", "90"),       
    ("SteeringRateCost", "700"),       
    ("LateralMotionCost", "11"),       
    ("LateralAccelCost", "0"),       
    ("LateralJerkCost", "5"),       
    ("LateralTorqueCustom", "0"),       
    ("LateralTorqueAccelFactor", "2500"),       
    ("LateralTorqueFriction", "100"),       
    ("NNFF", "0"),
    ("NNFFLite", "0"),
    ("ScreenAlwaysOn", "0"),
    ("BrakeResumeSpeedMode", "1"),
    ("ShowAdvancedSettings", "0"),
    ("SteerActuatorDelay", "30"),       
    ("CruiseControlMode", "0"),
    ("CruiseOnDist", "0"),
    ("SteerRatioApply", "0"),
    ("SteerDeltaUp", "3"),       
    ("SteerDeltaDown", "7"),       
  ]
  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')))

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # optech 주행 화면 (ShowHudMode 7, 2026-11): every device uses it (also forced below on every start; the
  # '화면 표시 방식' menu was removed 2026-11-23). Once: tyres on, as they are part of the right column
  if not params.get_bool("OptechHud7Set"):
    params.put("ShowHudMode", "7")
    params.put("ShowTpms", "1")      # tyres are part of the right column
    params.put_bool("OptechHud7Set", True)

  # Korean by default: once per install, switch to Korean even if an older install left English behind.
  # After that the language chosen in the menu is kept (the marker is removed only by a fresh install).
  langMarker = '/data/openpilot/.lang_ko_default'
  if not PC and os.path.isdir('/data/openpilot') and not os.path.isfile(langMarker):
    params.put("LanguageSetting", "main_ko")
    open(langMarker, 'w').close()

  # engage / disengage only by the driver's own button: automatic cruise ON/OFF from the gas pedal, brake
  # release, lead distance and auto engage are removed from the menu and forced off on every boot
  # KeepEngage too: leaving D (N / R) disengages, as in stock openpilot
  for k in ("EnableAutoEngage", "CruiseOnDist", "AutoResumeFromGas", "AutoCancelFromGasMode", "KeepEngage", "TrafficStopMode", "CruiseControlMode", "DisengageOnAccelerator"):
    params.put(k, "0")
  # apilot's own brake release resume (AutoResumeFromBrakeRelease) is pinned off below and removed from the menu;
  # cruise_helper resumes at the set speed instead (brake_paused). These conditions stay pinned for its code path:
  for k, v in (("AutoResumeFromBrakeReleaseLeadCar", "1"), ("AutoResumeFromBrakeReleaseDist", "20"),
               ("AutoResumeFromBrakeCarSpeed", "40"), ("AutoResumeFromBrakeReleaseTrafficSign", "0"),
               ("TFollowGap1", "110"), ("TFollowGap2", "120"), ("TFollowGap3", "140"), ("TFollowGap4", "160"),
               ("ApplyLongDynamicCost", "0"), ("TFollowSpeedRatio", "110")):
    params.put(k, v)
  # settings removed from the menu (4-tab cleanup): keep them at their defaults so they can't get stuck
  for k, v in (("MixRadarInfo", "0"), ("CarrotRadar", "0"), ("CarrotCutInPredecel", "0"), ("CarrotStoppingLead", "0"),
               ("AutoCurveSpeedFactorIn", "10"), ("CarrotTurnAccel", "0"),
               ("AutoNaviSpeedDecelRate", "80"),
               ("AutoNaviSpeedCtrlMode", "0"), ("AutoNaviSpeedBumpDist", "10"),
               ("AutoTurnControl", "0"), ("AutoTurnControlSpeedLaneChange", "60"), ("AutoTurnControlSpeedTurn", "20"),
               ("AutoTurnControlTurnEnd", "3"),
               ("LongitudinalTuningKpV", "100"), ("LongitudinalTuningKiV", "200"), ("LongitudinalTuningKf", "100"),
               ("SteeringRateCost", "700"), ("PathCostApply", "100"), ("LateralAccelCost", "0"),
               ("LateralMotionCost", "11"), ("LateralJerkCost", "5"), ("MaxAngleFrames", "89"),
               ("SteerDeltaUp", "3"), ("SteerDeltaDown", "7"), ("UseLaneLineSpeed", "0"),
               ("CruiseButtonMode", "0"), ("CruiseSpeedMin", "10"),
               ("ShowHudMode", "7"),
               ("GpsSource", "1"),
               # brake release resume is done by cruise_helper (brake_paused): the old apilot conditions are off
               ("AutoResumeFromBrakeRelease", "0"), ("BrakeResumeSpeedMode", "1")):  # GPS is always the phone built-in one; the selector is removed  # screen mode fixed to the optech 주행 화면 (7); the selector is removed
    params.put(k, v)

  # optech: 감속 시작 / 감속 끝 / 목표 속도 are menu settings again (주행 > 카메라 감속); they used to be reset here on
  # every boot. Once: start them at 22 s / 6 s / 100 % (driver feedback: pass the camera at the posted limit)
  if params.get("OptechCamTuneSet") is None:
    params.put("AutoNaviSpeedCtrlStart", "22")
    params.put("AutoNaviSpeedCtrlEnd", "6")
    params.put("AutoNaviSpeedSafetyFactor", "100")
    params.put("OptechCamTuneSet", "1")

  optech_navi_link_migrate(params)

  # is this dashcam?
  if os.getenv("PASSIVE") is not None:
    params.put_bool("Passive", bool(int(os.getenv("PASSIVE", "0"))))

  if params.get("Passive") is None:
    raise Exception("Passive must be set to continue")

  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set version params
  params.put("Version", get_version())
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", get_commit(default=""))
  # release date of the code (commit time, as written by the release), shown as 브랜치 커밋 on the software page
  try:
    # always in Korea time, whatever time zone the commit was made in (POSIX TZ string, no tzdata needed)
    commit_date = subprocess.check_output(["git", "log", "-1", "--format=%cd", "--date=format-local:%Y-%m-%d %H:%M:%S"],
                                          cwd=BASEDIR, encoding="utf8", stderr=subprocess.DEVNULL,
                                          env=dict(os.environ, TZ="KST-9")).strip()
  except Exception:
    commit_date = ""
  params.put("GitCommitDate", commit_date)
  params.put("GitBranch", get_short_branch(default=""))
  params.put("GitRemote", get_origin(default=""))

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    serial = params.get("HardwareSerial")
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog

  if not is_dirty():
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id, version=get_version(), dirty=is_dirty(),
                       device=HARDWARE.get_device_type())
  if os.path.isfile('/data/tmux_error.log'):
    os.remove('/data/tmux_error.log')


def manager_prepare() -> None:
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:

  if EON:
    Process(name="autoshutdownd", target=launcher, args=("selfdrive.autoshutdownd", "autoshutdownd")).start()
    # optech 고급 > 주행 기록 내려받기: serves /data/drivelog to the phone for 15 min after the menu button
    Process(name="logserver", target=launcher, args=("selfdrive.carrot.tools.logserver", "logserver")).start()
    system("am startservice com.neokii.optool/.MainService")

  Process(name="road_speed_limiter", target=launcher, args=("selfdrive.road_speed_limiter", "road_speed_limiter")).start()
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: List[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  ensure_running(managed_processes.values(), started=False, not_run=ignore)

  started_prev = False
  sm = messaging.SubMaster(['deviceState'])
  pm = messaging.PubMaster(['managerState'])

  print_timer = 0
  while True:
    sm.update()
    not_run = ignore[:]

    started = sm['deviceState'].started
    driverview = params.get_bool("IsDriverViewEnabled")
    ensure_running(managed_processes.values(), started, driverview, not_run)

    # trigger an update after going offroad
    if started_prev and not started and 'updated' in managed_processes:
      os.sync()
      managed_processes['updated'].signal(signal.SIGHUP)

    started_prev = started

    running = ' '.join("%s%s\u001b[0m" % ("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print_timer = (print_timer + 1)%10
    if print_timer == 0:
      print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState')
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", param)
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  # always fast boot: build.py just finished, so mark this commit as built.
  # launch_chffrplus.sh drops the mark when the code changes (git pull / reinstall).
  preBuiltFile = '/data/openpilot/prebuilt'
  just_built = not os.path.isfile(preBuiltFile)
  if os.path.isdir("/data/openpilot") and just_built:
    os.system("cd /data/openpilot; git rev-parse HEAD > .prebuilt_commit && touch prebuilt")

  prepare_only = os.getenv("PREPAREONLY") is not None

  manager_init()
  # car lists for the car select menu: each run is a full python start (a few seconds on C2),
  # so only rebuild them when the code changed (just built) or a list is missing
  carList = '/data/params/d/SupportedCars'
  if just_built or not (os.path.isfile(carList) and os.path.getsize(carList) > 0):
    os.system("python /data/openpilot/selfdrive/car/hyundai/values.py > " + carList)
  # The old separate GM list file is gone: GM cars that run on this branch (Bolt EUV) are in SupportedCars itself
  # (selfdrive/car/hyundai/values.py main). A car chosen from that old file is cleared so the car is fingerprinted.
  gmList = '/data/params/d/SupportedCars_gm'
  if os.path.isfile(gmList):
    try:
      params = Params()
      selected = params.get("SelectedCar", encoding="utf8")
      if selected and selected in open(gmList, encoding="utf8").read().splitlines():
        params.remove("SelectedCar")
      os.remove(gmList)
    except Exception:
      pass

  # Start UI early so prepare can happen in the background
  if not prepare_only:
    managed_processes['ui'].start()

  manager_prepare()

  if prepare_only:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
