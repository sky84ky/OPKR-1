#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.numpy_fast import interp
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import NON_SCC_CAR, NON_SCC_VARIANT, HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, Buttons, CarControllerParams
from selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from selfdrive.car import STD_CARGO_KG, create_button_event, scale_tire_stiffness, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.disable_ecu import disable_ecu
from common.params import Params
from selfdrive.controls.lib.cruise_helper import enable_radar_tracks

Ecu = car.CarParams.Ecu
ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    # a model chosen with SCC that shows no SCC11 (1056) on either bus is its trim without SCC (values.NON_SCC_VARIANT)
    if candidate in NON_SCC_VARIANT and len(fingerprint[0]) and 1056 not in fingerprint[0] and 1056 not in fingerprint[2]:
      candidate = NON_SCC_VARIANT[candidate]
      ret.carFingerprint = candidate
    ret.carName = "hyundai"
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = candidate in {CAR.KIA_OPTIMA_H, }

    if candidate in CANFD_CAR:
      # detect HDA2 with ADAS Driving ECU
      if Ecu.adas in [fw.ecu for fw in car_fw]:
        ret.flags |= HyundaiFlags.CANFD_HDA2.value
      else:
        # non-HDA2
        if 0x1cf not in fingerprint[4]:
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
        # ICE cars do not have 0x130; GEARS message on 0x40 instead
        if 0x130 not in fingerprint[4]:
          ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value
        if candidate not in CANFD_RADAR_SCC_CAR:
          ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value
    else:
      # Send LFA message on cars with HDA
      if 0x485 in fingerprint[2]:
        ret.flags |= HyundaiFlags.SEND_LFA.value
      # 기타 차종 without SCC (Ray EV, Porter II EV): stock cruise control only (StarPilot / carrot)
      if candidate in NON_SCC_CAR:
        ret.flags |= HyundaiFlags.NON_SCC.value
        if 916 not in fingerprint[0]:  # TCS13
          ret.flags |= HyundaiFlags.NON_SCC_NO_TCS.value
        # the Ray EV's LFAHDA_MFC (0x485) is 8 bytes; ours is the 4-byte one, so do not send it there (StarPilot keeps a
        # separate Ray DBC for it)
        if fingerprint[2].get(0x485) != 4:
          ret.flags &= ~HyundaiFlags.SEND_LFA.value

      # These cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
      if 0x38d in fingerprint[0] or 0x38d in fingerprint[2]:
        ret.flags |= HyundaiFlags.USE_FCA.value


    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerLimitTimer = 0.4

    ret.steerRatio = 16.
    tire_stiffness_factor = 1.
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate in (CAR.SANTA_FE, CAR.SANTA_FE_2022, CAR.SANTA_FE_HEV_2022, CAR.SANTA_FE_PHEV_2022):
      ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      tire_stiffness_factor = 0.82
    elif candidate in (CAR.SONATA, CAR.SONATA_HYBRID):
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
    elif candidate == CAR.SONATA_LF:
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.PALISADE:
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      #ret.steerRatio = 15.6 * 1.15
      ret.steerRatio = 13.96 # 지구별.
      tire_stiffness_factor = 0.63
    elif candidate in (CAR.ELANTRA, CAR.ELANTRA_GT_I30):
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      tire_stiffness_factor = 0.385    # stiffnessFactor settled on 1.0081302973865127
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate in (CAR.ELANTRA_2021, CAR.ELANTRA_2022_NON_SCC):
      ret.mass = (2800. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 12.9
      tire_stiffness_factor = 0.65
    elif candidate in (CAR.ELANTRA_HEV_2021, CAR.ELANTRA_HEV_2022_NON_SCC):
      ret.mass = (3017. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 12.9
      tire_stiffness_factor = 0.65
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      #ret.minSteerSpeed = 60 * CV.KPH_TO_MS
    elif candidate == CAR.KONA_2022:
      ret.mass = 1491. + STD_CARGO_KG
      ret.wheelbase = 2.6
      ret.steerRatio = 13.42
      tire_stiffness_factor = 0.385
    elif candidate in (CAR.KONA, CAR.KONA_NON_SCC, CAR.KONA_EV, CAR.KONA_EV_NON_SCC, CAR.KONA_HEV, CAR.KONA_EV_2022):
      ret.mass = {CAR.KONA_EV: 1685., CAR.KONA_HEV: 1425., CAR.KONA_EV_2022: 1743.}.get({CAR.KONA_EV_NON_SCC: CAR.KONA_EV}.get(candidate, candidate), 1275.) + STD_CARGO_KG
      ret.wheelbase = 2.6
      ret.steerRatio = 13.42  # Spec
      tire_stiffness_factor = 0.385
    elif candidate in (CAR.IONIQ, CAR.IONIQ_EV_LTD, CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.IONIQ_HEV_2022):
      ret.mass = 1490. + STD_CARGO_KG  # weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
      if candidate not in (CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.IONIQ_HEV_2022):
        #ret.minSteerSpeed = 32 * CV.MPH_TO_MS
        pass
    elif candidate == CAR.IONIQ_PHEV_2019:
      ret.mass = 1550. + STD_CARGO_KG  # weight per hyundai site https://www.hyundaiusa.com/us/en/vehicles/2019-ioniq-plug-in-hybrid/compare-specs
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.VELOSTER:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
      tire_stiffness_factor = 0.5
    elif candidate == CAR.TUCSON:
      ret.mass = 3520. * CV.LB_TO_KG
      ret.wheelbase = 2.67
      ret.steerRatio = 14.00 * 1.15
      tire_stiffness_factor = 0.385
    elif candidate == CAR.TUCSON_TL_SCC:
      ret.mass = 1594. + STD_CARGO_KG #1730
      ret.wheelbase = 2.67
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerRatio = 14.00
    elif candidate == CAR.TUCSON_HYBRID_4TH_GEN:
      ret.mass = 1680. + STD_CARGO_KG  # average of all 3 trims
      ret.wheelbase = 2.756
      ret.steerRatio = 16.
      tire_stiffness_factor = 0.385
    elif candidate == CAR.SANTA_CRUZ_1ST_GEN:
      ret.mass = 1870. + STD_CARGO_KG  # weight from Limited trim - the only supported trim
      ret.wheelbase = 3.000
      ret.steerRatio = 14.2  # steering ratio according to Hyundai News https://www.hyundainews.com/assets/documents/original/48035-2022SantaCruzProductGuideSpecsv2081521.pdf

    # Kia
    elif candidate == CAR.KIA_SORENTO:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
    elif candidate in (CAR.KIA_NIRO_EV, CAR.KIA_NIRO_PHEV, CAR.KIA_NIRO_HEV_2021, CAR.KIA_NIRO_PHEV_2022):
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.9 if CAR.KIA_NIRO_HEV_2021 else 13.73  # Spec
      tire_stiffness_factor = 0.385
      if candidate == CAR.KIA_NIRO_PHEV:
        #ret.minSteerSpeed = 32 * CV.MPH_TO_MS
        pass
    elif candidate in (CAR.KIA_SELTOS, CAR.KIA_SELTOS_2023_NON_SCC):
      ret.mass = 1337. + STD_CARGO_KG
      ret.wheelbase = 2.63
      ret.steerRatio = 14.56
      tire_stiffness_factor = 1
    elif candidate == CAR.KIA_SPORTAGE_5TH_GEN:
      ret.mass = 1700. + STD_CARGO_KG  # weight from SX and above trims, average of FWD and AWD versions
      ret.wheelbase = 2.756
      ret.steerRatio = 13.6  # steering ratio according to Kia News https://www.kiamedia.com/us/en/models/sportage/2023/specifications
    elif candidate in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_OPTIMA_H_G4_FL):
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
      if candidate == CAR.KIA_OPTIMA_G4:
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate in (CAR.KIA_STINGER, CAR.KIA_STINGER_2022):
      ret.mass = 1825. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
    elif candidate in (CAR.KIA_FORTE, CAR.KIA_FORTE_2021_NON_SCC, CAR.KIA_FORTE_2019_NON_SCC):
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_CEED:
      ret.mass = 1450. + STD_CARGO_KG
      ret.wheelbase = 2.65
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
    elif candidate in (CAR.KIA_K5_2021, CAR.KIA_K5_HEV_2020):
      ret.mass = 3228. * CV.LB_TO_KG
      ret.wheelbase = 2.85
      ret.steerRatio = 13.27  # 2021 Kia K5 Steering Ratio (all trims)
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_EV6:
      ret.mass = 2055 + STD_CARGO_KG
      ret.wheelbase = 2.9
      ret.steerRatio = 16.
      tire_stiffness_factor = 0.65
    elif candidate == CAR.IONIQ_5:
      ret.mass = 2012 + STD_CARGO_KG
      ret.wheelbase = 3.0
      ret.steerRatio = 16.
      tire_stiffness_factor = 0.65
    elif candidate == CAR.KIA_SPORTAGE_HYBRID_5TH_GEN:
      ret.mass = 1767. + STD_CARGO_KG  # SX Prestige trim support only
      ret.wheelbase = 2.756
      ret.steerRatio = 13.6
    elif candidate == CAR.KIA_SOUL_EV_SK3:
      ret.steerRatio = 13.7  # average of the platforms
      ret.mass = 1375. + STD_CARGO_KG
      ret.wheelbase = 2.6      

    # Genesis
    elif candidate == CAR.GENESIS_G70:
      ret.steerActuatorDelay = 0.1
      ret.mass = 1640.0 + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.56
    elif candidate in (CAR.GENESIS_G70_2020, CAR.GENESIS_G70_2021_NON_SCC):
      ret.mass = 3673.0 * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 12.9
    elif candidate == CAR.GENESIS_GV70_1ST_GEN:
      ret.mass = 1950. + STD_CARGO_KG
      ret.wheelbase = 2.87
      ret.steerRatio = 14.6
    elif candidate == CAR.GENESIS_G80:
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.steerRatio = 12.069

    elif candidate in [CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_HEV]:
      ret.mass = 1570. + STD_CARGO_KG
      ret.wheelbase = 2.845
      ret.steerRatio = 16.
      tire_stiffness_factor = 0.8
      ret.centerToFront = ret.wheelbase * 0.385
    elif candidate in (CAR.GRANDEUR_IG_FL, CAR.GRANDEUR_IG_FL_HEV):
      ret.mass = 1675. + STD_CARGO_KG
      ret.wheelbase = 2.885
      ret.steerRatio = 16.5  
      tire_stiffness_factor = 0.8
      ret.steerActuatorDelay = 0.2
    elif candidate == CAR.NEXO: # fix PolorBear - 22.06.05
      ret.mass = 1885. + STD_CARGO_KG
      ret.wheelbase = 2.79
      ret.steerRatio = 15.3
      tire_stiffness_factor = 0.385
    elif candidate == CAR.CASPER:
      ret.mass = 1060. + STD_CARGO_KG
      ret.wheelbase = 2.4
      ret.steerRatio = 14.3
      #tire_stiffness_factor = 0.385
    elif candidate == CAR.CASPER_EV:
      ret.mass = 1355. + STD_CARGO_KG
      ret.wheelbase = 2.58
      ret.steerRatio = 14.3
    elif candidate == CAR.CUSTIN_1ST_GEN:
      ret.mass = 1690. + STD_CARGO_KG
      ret.wheelbase = 3.055
      ret.steerRatio = 17.
    elif candidate == CAR.KIA_MOHAVE:
      ret.mass = 2285. + STD_CARGO_KG
      ret.wheelbase = 2.895
      ret.steerRatio = 16.
      tire_stiffness_factor = 0.7
    elif candidate in (CAR.KIA_K5_JF, CAR.KIA_K5_HEV_JF, CAR.KIA_K5_HEV_2022):
      ret.mass = {CAR.KIA_K5_HEV_JF: 1705.}.get(candidate, 1515.) + STD_CARGO_KG
      ret.wheelbase = 2.85 if candidate == CAR.KIA_K5_HEV_2022 else 2.80
      ret.steerRatio = 15.5
      tire_stiffness_factor = 0.7
    elif candidate == CAR.KIA_K9:
      ret.mass = 2075. + STD_CARGO_KG
      ret.wheelbase = 3.15
      ret.steerRatio = 14.5
      tire_stiffness_factor = 0.7
    elif candidate == CAR.VENUE_2021_NON_SCC:  # sunnypilot (wheelbase, steer ratio); curb weight about 1.2 t
      ret.mass = 1200. + STD_CARGO_KG
      ret.wheelbase = 2.52
      ret.steerRatio = 14.4
      tire_stiffness_factor = 0.65
    elif candidate == CAR.KIA_RAY_EV:
      ret.mass = 1295. + STD_CARGO_KG
      ret.wheelbase = 2.52
      ret.steerRatio = 14.5
      tire_stiffness_factor = 0.65
    elif candidate == CAR.PORTER_II_EV:
      ret.mass = 1970. + STD_CARGO_KG
      ret.wheelbase = 2.64
      ret.steerRatio = 14.5
      tire_stiffness_factor = 0.65
    elif candidate in [CAR.K7, CAR.K7_HEV, CAR.K7_PE, CAR.K7_HEV_PE]:
      ret.mass = 1850. + STD_CARGO_KG
      ret.wheelbase = 2.855
      ret.steerRatio = 15.5
      tire_stiffness_factor = 0.7
    elif candidate == CAR.GENESIS_EQ900:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.steerRatio = 16.0
      ret.steerActuatorDelay = 0.075
    elif candidate == CAR.GENESIS_EQ900_L:
      ret.mass = 2290
      ret.wheelbase = 3.45
    elif candidate == CAR.GENESIS_G90_2019:
      ret.mass = 2150
      ret.wheelbase = 3.16
      

    # *** longitudinal control ***
    if candidate in CANFD_CAR:
      ret.longitudinalTuning.kpV = [0.1]
      ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = candidate in (HYBRID_CAR | EV_CAR) and candidate not in CANFD_RADAR_SCC_CAR
    else:
      ret.longitudinalTuning.kpV = [0.5]
      ret.longitudinalTuning.kiV = [0.0]
     

      print("fingerprint:", fingerprint)

      # 판다코드가 수정되어 자동인식 안됨~
      ret.enableBsm = 0x58b in fingerprint[0]
      ret.sccBus = 2 if 1056 in fingerprint[2] else 0 if 1056 in fingerprint[0] else -1

      ret.naviCluster = 1 if 1348 in fingerprint[0] else 0

      ret.sccBus = 0
      if Params().get_bool("SccConnectedBus2") and candidate not in NON_SCC_CAR:
        ret.sccBus = 2
      
      print("***************************************************************************")
      print("sccBus = ", ret.sccBus)
      #if ret.sccBus == 2:  필요없는 코드네~~
      #  ret.openpilotLongitudinalControl = True

      ret.hasLfaHda = 1157 in fingerprint[0] or 1157 in fingerprint[2]

      # SCC버스가 2이면 무조건 롱컨~
      if ret.sccBus == 2 and not candidate in CAMERA_SCC_CAR:
        experimental_long = True
      # SCC버스가 0인데 롱컨이면, EnableRadarTracks무조건 켜기...
      elif ret.sccBus == 0 and Params().get_bool("EnableRadarTracks"):
        experimental_long = True
      else:
        experimental_long = False

      ret.experimentalLongitudinalAvailable = candidate not in (LEGACY_SAFETY_MODE_CAR | CAMERA_SCC_CAR)
      ret.experimentalLongitudinalAvailable = True
      if candidate in NON_SCC_CAR:  # no SCC to command: steering only, the car's cruise control keeps the speed
        experimental_long = False
        ret.experimentalLongitudinalAvailable = False
    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    # cars without SCC with a comma pedal: the car's own menu switch ('페달 사용') and the pedal box on the bus (0x201, 6 bytes)
    from selfdrive.car.hyundai.pedal import PEDAL_PARAMS
    if candidate in PEDAL_PARAMS:  # Ray EV / Porter II EV only; the other cars without SCC have no pedal setup
      from selfdrive.car.hyundai.pedal import PEDAL_SENSOR_ADDR, PEDAL_SENSOR_LEN
      if Params().get_bool(PEDAL_PARAMS[candidate][0]) and fingerprint[0].get(PEDAL_SENSOR_ADDR) == PEDAL_SENSOR_LEN:
        ret.flags |= HyundaiFlags.NON_SCC_PEDAL.value
        ret.enableGasInterceptor = True
        ret.experimentalLongitudinalAvailable = True
        ret.openpilotLongitudinalControl = True
        ret.pcmCruise = False
        ret.radarUnavailable = True  # no radar: the camera finds the lead
        ret.minEnableSpeed = -1.
        ret.autoResumeSng = False    # no brake to hold the car: no stop and go
        # carrot Ray EV pedal long: this base's long control is a PID on the speed error like carrot's (StarPilot's
        # 0.05~0.1 gains were for an accel error), so Kp 1, Kf 1 and the Ray menu's Ki. Two breakpoints so the
        # global long tuning menu (single breakpoint only, longcontrol.py) does not replace it.
        from selfdrive.car.hyundai.pedal import KI_PARAM, STRONG_PARAM, ray_ki
        ki = ray_ki(Params().get(KI_PARAM, encoding="utf8")) if candidate == CAR.KIA_RAY_EV else 0.05
        ret.longitudinalTuning.kpBP = [0., 35.]
        ret.longitudinalTuning.kpV = [1.0, 1.0]
        ret.longitudinalTuning.kiBP = [0., 35.]
        ret.longitudinalTuning.kiV = [ki, ki]
        ret.longitudinalTuning.kf = 1.0
        if candidate == CAR.KIA_RAY_EV and Params().get_bool(STRONG_PARAM):
          ret.flags |= HyundaiFlags.NON_SCC_PEDAL_STRONG.value

    ret.stoppingControl = True
    ret.startingState = False # True # startAccel을 적용하는 startingState를 막음.
    ret.vEgoStarting = 0.2
    ret.vEgoStopping = 0.3
    ret.startAccel = 2.0
    ret.stoppingDecelRate = 1.2 # brake_travel/s while trying to stop
    ret.longitudinalActuatorDelayLowerBound = 0.5
    ret.longitudinalActuatorDelayUpperBound = 0.5

    #ret.radarTimeStep = (1.0 / 50) # 50Hz   SCC11, RadarTrack은 50Hz
    ret.radarTimeStep = (1.0 / 20) # 20Hz 레이더부하줄일겸..

    # *** feature detection ***
    if candidate in CANFD_CAR:
      bus = 5 if ret.flags & HyundaiFlags.CANFD_HDA2 else 4
      ret.enableBsm = 0x1e5 in fingerprint[bus]
    else:
      ret.enableBsm = 0x58b in fingerprint[0]

    # *** panda safety config ***
    if candidate in CANFD_CAR:
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput),
                           get_safety_config(car.CarParams.SafetyModel.hyundaiCanfd)]

      if ret.flags & HyundaiFlags.CANFD_HDA2:
        ret.safetyConfigs[1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        ret.safetyConfigs[1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_ALT_BUTTONS
      if ret.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        ret.safetyConfigs[1].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC
    else:
      if candidate in NON_SCC_CAR:
        # no SCC12/TCS checks to wait for: the legacy mode with the non-SCC flag (panda safety_hyundai.h)
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_NON_SCC
        if ret.flags & HyundaiFlags.NON_SCC_PEDAL:
          ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_NON_SCC_PEDAL
          if ret.flags & HyundaiFlags.NON_SCC_PEDAL_STRONG:
            ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_NON_SCC_PEDAL_STRONG
      elif candidate in LEGACY_SAFETY_MODE_CAR:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]

      if candidate in CAMERA_SCC_CAR:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC
      if ret.sccBus == 2:
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]

    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_LONG
    if candidate in HYBRID_CAR:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_HYBRID_GAS
    elif candidate in EV_CAR:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_EV_GAS

    if candidate in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022, CAR.KONA_NON_SCC, CAR.KONA_EV_NON_SCC):
      ret.flags |= HyundaiFlags.ALT_LIMITS.value
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_ALT_LIMITS

    if Params().get_bool("EnableAutoEngage"):
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_AUTO_ENGAGE

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    # optech 조향 탭 '조향 방식' (OptechLatMethod, applied at the next start): 0 토크 (default, set above),
    # 1 PID, 2 INDI, 3 LQR with OPKR's Hyundai/Kia default values (opkr-c2 tunes.py / manager.py), adjustable
    if ret.steerControlType != car.CarParams.SteerControlType.angle:
      set_optech_lat_method(ret.lateralTuning)
    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.flags & HyundaiFlags.NON_SCC:
      return  # no SCC radar to switch off or to ask for tracks (the pedal cars run openpilot long without one)
    if CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and CP.sccBus == 0:
      addr, bus = 0x7d0, 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, 5
      disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
      enable_radar_tracks(CP, logcan, sendcan)
      Params().put_bool("EnableRadarTracks", True)
    elif Params().get_bool("EnableRadarTracks"): #롱컨관계없이 EnableRadarTracks는 사용할 수 있을듯..
      enable_radar_tracks(CP, logcan, sendcan)      

  def _update(self, c):
    self.CS.cp_pedal = self.cp_body  # comma pedal GAS_SENSOR parser (None unless NON_SCC_PEDAL)
    ret = self.CS.update(self.cp, self.cp_cam)

    #if self.CS.CP.openpilotLongitudinalControl and self.CS.cruise_buttons[-1] != self.CS.prev_cruise_buttons:
    if self.CS.cruise_buttons[-1] != self.CS.prev_cruise_buttons:
      buttonEvents = [create_button_event(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT)]
      # Handle CF_Clu_CruiseSwState changing buttons mid-press
      if self.CS.cruise_buttons[-1] != 0 and self.CS.prev_cruise_buttons != 0:
        buttonEvents.append(create_button_event(0, self.CS.prev_cruise_buttons, BUTTONS_DICT))

      ret.buttonEvents = buttonEvents

    # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
    # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
    # Main button also can trigger an engagement on these cars
    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)

    # carrot's cruise button rule (2026-11-27), stock SCC: with the cruise standby on and openpilot off (CANCEL held, or
    # a standby that comes on by itself at start), the driver's SET/RES turn the car's SCC on -> openpilot engages too,
    # with speed control (controlsd engage_long_user). It used to cancel the car's SCC at once. Openpilot long cars
    # engage on the SET/RES release (buttonEnable) as before.
    if self.CS.CP.pcmCruise and ret.cruiseState.enabled and not self.CS.out.cruiseState.enabled and allow_enable \
       and not c.enabled:
      events.add(car.CarEvent.EventName.pcmEnable)

    # C2 port (one-button use): the cruise main button (CF_Clu_CruiseSwMain) pressed while openpilot is engaged
    # disengages steering and speed control together. Engaging is unchanged (the SCC turning on gives pcmEnable;
    # controlsd then starts speed control at once). The II/cancel button keeps its apilot behavior.
    main_btn = self.CS.main_buttons[-1]
    if main_btn and not getattr(self, "main_btn_prev", 0) and c.enabled:
      events.add(car.CarEvent.EventName.buttonCancel)
    self.main_btn_prev = main_btn

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    return self.CC.update(c, self.CS)

def _optech_param(params, key, default, scale):
  try:
    return int(params.get(key, encoding="utf8") or default) * scale
  except (TypeError, ValueError):
    return default * scale


def set_optech_lat_method(tune):
  params = Params()
  try:
    method = int(params.get("OptechLatMethod", encoding="utf8") or 0)
  except (TypeError, ValueError):
    method = 0
  if method == 1:
    tune.init('pid')
    tune.pid.kpBP, tune.pid.kpV = [0., 9.], [0.1, _optech_param(params, "OptechPidKp", 25, 0.01)]
    tune.pid.kiBP, tune.pid.kiV = [0., 9.], [0.01, _optech_param(params, "OptechPidKi", 40, 0.001)]
    tune.pid.kf = _optech_param(params, "OptechPidKf", 7, 0.00001)
  elif method == 2:
    tune.init('indi')
    tune.indi.innerLoopGainBP, tune.indi.innerLoopGainV = [0.], [_optech_param(params, "OptechIndiInner", 35, 0.1)]
    tune.indi.outerLoopGainBP, tune.indi.outerLoopGainV = [0.], [_optech_param(params, "OptechIndiOuter", 20, 0.1)]
    tune.indi.timeConstantBP, tune.indi.timeConstantV = [0.], [_optech_param(params, "OptechIndiTime", 14, 0.1)]
    tune.indi.actuatorEffectivenessBP, tune.indi.actuatorEffectivenessV = [0.], [_optech_param(params, "OptechIndiAct", 20, 0.1)]
  elif method == 3:
    tune.init('lqr')
    tune.lqr.scale = _optech_param(params, "OptechLqrScale", 1500, 1.0)
    tune.lqr.ki = _optech_param(params, "OptechLqrKi", 16, 0.001)
    tune.lqr.dcGain = _optech_param(params, "OptechLqrDcGain", 265, 0.00001)
    tune.lqr.a = [0., 1., -0.22619643, 1.21822268]
    tune.lqr.b = [-1.92006585e-04, 3.95603032e-05]
    tune.lqr.c = [1., 0.]
    tune.lqr.k = [-110., 451.]
    tune.lqr.l = [0.33, 0.318]
