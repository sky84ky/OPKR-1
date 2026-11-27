#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <string>
#include <memory>

#include <QDebug>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/common/params.h"
#include "selfdrive/common/watchdog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/input.h"

#include <QComboBox>
#include <QAbstractItemView>
#include <QScroller>
#include <QListView>
#include <QListWidget>
#include <QMap>

#include <QProcess> // opkr
#include <QDateTime> // opkr
#include <QTimer> // opkr
#include <QFileInfo> // opkr
#include <QDir>
#include <algorithm>
#include <QNetworkInterface>  // optech 주행 기록 내려받기
#include <QImage>
#include <QPixmap>
#include <QrCode.hpp>
#include <QLabel>

// optech '운전자 모니터링' (OptechDmMode, applied at reboot): 0 기본 사용 (camera, dmonitoringd), 1 경도 사용 (no camera,
// dpmonitoringd: 5 min without any steering / pedal / button), 2 사용 안 함 (dpmonitoringd sends no warnings).
// ShowDmInfo follows it for process_config / boardd: -1 camera off, 0 / 1 icon hidden / shown
static int optech_dm_mode() {
  const std::string v = Params().get("OptechDmMode");
  if (v.empty()) return std::atoi(Params().get("ShowDmInfo").c_str()) < 0 ? 1 : 0;  // set before this menu existed
  return std::clamp(std::atoi(v.c_str()), 0, 2);
}

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  // param, title, desc, icon, confirm
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      tr("Enable openpilot"),
      tr("Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off."),
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "ExperimentalMode",
      tr("Experimental Mode"),
      "",
      "../assets/img_experimental_white.svg",
    },
    {
      "ExperimentalLongitudinalEnabled",
      tr("openpilot 가감속 제어 (알파)"),
      QString("<b>%1</b><br><br>%2")
      .arg(tr("경고: 이 차의 openpilot 가감속 제어는 알파 단계이며, 자동 긴급 제동(AEB)이 꺼집니다."))
      .arg(tr("이 차에서는 openpilot이 기본적으로 차의 순정 ACC를 사용합니다. 켜면 openpilot 가감속 제어로 바뀝니다. openpilot 가감속 제어(알파)를 켤 때는 실험 모드도 켜는 것을 권장합니다.")),
      "../assets/offroad/icon_speed_limit.png",
    },
    {
      "SccConnectedBus2",
      "롱컨 (Long Control)",
      "롱컨 작업을 마치셨으면 켜 주세요.",
      "../assets/offroad/icon_warning.png",
    },
    {
      "EnableRadarTracks",
      "레이더 트랙 사용",
      "SCC 데이터 대신 레이더 트랙을 사용합니다. (산타페 2022 HEV)",
      "../assets/offroad/icon_warning.png",
    },
    {
      // OPKR 'LDWS 차량 설정' (LdwsCarFix): hyundaican.create_lkas11 sends CF_Lkas_LdwsOpt_USM = 3
      "OptechLdwsCar",
      "LDWS 차량 (차로 유지 보조 없음)",
      "차선 이탈 경고(LDWS)만 있고 차로 유지 보조(LKAS)가 없는 차만 켜세요. "
      "이런 차에서 조향이 안 되거나 계기판에 차로 유지 관련 오류가 뜰 때 씁니다 (OPKR의 'LDWS 차량 설정'과 같습니다). "
      "LKAS가 있는 차는 켜지 마세요. 재부팅해야 적용됩니다.",
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsLdwEnabled",
      tr("Enable Lane Departure Warnings"),
      tr("Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h)."),
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsMetric",
      tr("Use Metric System"),
      tr("Display speed in km/h instead of mph."),
      "../assets/offroad/icon_metric.png",
    },
    {
      "RecordFront",
      tr("Record and Upload Driver Camera"),
      tr("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
      "../assets/offroad/icon_monitoring.png",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      tr("Show ETA in 24h Format"),
      tr("Use 24h format instead of am/pm"),
      "../assets/offroad/icon_metric.png",
    },
    {
      "NavSettingLeftSide",
      tr("Show Map on Left Side of UI"),
      tr("Show map on left side when in split screen view."),
      "../assets/offroad/icon_road.png",
    },
#endif
  };

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);

    bool locked = false;// params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);

    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  // Toggles with confirmation dialogs
  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, true);
  toggles["ExperimentalLongitudinalEnabled"]->setConfirmation(true, false);

  connect(toggles["ExperimentalLongitudinalEnabled"], &ToggleControl::toggleFlipped, [=]() {
    updateToggles();
  });
  // 운전자 모니터링: 기본 사용 / 경도 사용 / 사용 안 함 (the icon switch stays on OPTECH > 화면)
  {
    const QStringList dm_names = {"기본 사용", "경도 사용", "사용 안 함"};
    const QString dm_desc =
      "운전자가 운전에 집중하는지 확인하는 방식입니다.\n"
      "바꾼 뒤 기기를 다시 켜야 적용됩니다.\n"
      "\n"
      "[기본 사용] 카메라로 운전자 얼굴을 봅니다.\n"
      "· 앞을 안 보거나 졸면 1분 뒤 경고\n"
      "· 1분 2초: 경고음\n"
      "· 1분 8초: 빨간 경고와 감속\n"
      "\n"
      "[경도 사용] 카메라를 쓰지 않습니다. (기기 발열이 줄어듭니다)\n"
      "· 핸들·페달·버튼을 5분 동안 전혀 안 쓰면 경고와 감속\n"
      "· 하나라도 쓰면 처음부터 다시 셉니다\n"
      "\n"
      "[사용 안 함] 카메라도, 경고도 없습니다.\n"
      "· 운전자가 반응이 없어도 계속 운전하므로 사용을 추천하지 않습니다\n"
      "\n"
      "[감속에 대해]\n"
      "· 롱컨 차량: 완전히 멈출 때까지 천천히 감속합니다\n"
      "· 순정 크루즈(SCC) 차량: 설정 속도를 시속 30km까지 낮춥니다\n"
      "  순정 크루즈는 30km 아래로 설정할 수 없어 완전히 멈추지는 않습니다\n"
      "  경고가 풀려도 설정 속도는 낮춘 그대로이니 직접 다시 올리세요\n"
      "· 비상등은 켜지지 않습니다";
    auto dm_btn = new ButtonControl("", "선택", dm_desc);
    auto dm_title = [=]() { dm_btn->setTitle("운전자 모니터링: " + dm_names[optech_dm_mode()]); };
    dm_title();
    QObject::connect(dm_btn, &ButtonControl::clicked, [=]() {
      // show the explanation first, then the choice
      ConfirmationDialog info("<b>운전자 모니터링</b><br><br>" + dm_desc.toHtmlEscaped().replace("\n", "<br>"),
                              "선택하기", "취소", true, this);
      if (!info.exec()) return;
      const QString cur = dm_names[optech_dm_mode()];
      const QString sel = MultiOptionDialog::getSelection("운전자 모니터링", dm_names, cur, this);
      if (sel.isEmpty() || sel == cur) return;
      const int m = dm_names.indexOf(sel);
      Params().put("OptechDmMode", std::to_string(m));
      Params().put("ShowDmInfo", m == 0 ? "1" : "-1");
      dm_title();
      ConfirmationDialog::alert("기기를 다시 켜야 적용됩니다.", this);
    });
    addItem(dm_btn);
  }
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto e2e_toggle = toggles["ExperimentalMode"];
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
  const QString e2e_description = QString("%1<br>"
                                          "<h4>%2</h4><br>"
                                          "%3<br>"
                                          "<h4>%4</h4><br>"
                                          "%5")
                                  .arg(tr("openpilot은 기본적으로 <b>안정 모드</b>로 주행합니다. 실험 모드는 아직 안정 모드에 넣기 어려운 <b>알파 단계 기능</b>을 켭니다. 실험 기능은 다음과 같습니다:"))
                                  .arg(tr("엔드투엔드 가감속 제어"))
                                  .arg(tr("주행 모델이 가속과 브레이크를 직접 제어합니다. 빨간 신호와 정지 표지판에서 멈추는 것까지 사람이 운전하듯 판단합니다. "
                                       "속도는 주행 모델이 정하기 때문에 크루즈 설정 속도는 최고 속도 제한으로만 쓰입니다. 알파 단계 기능이라 실수가 있을 수 있습니다."))
                                  .arg(tr("새 주행 화면"))
                                  .arg(tr("저속에서는 회전 구간을 더 잘 보여주기 위해 주행 화면이 광각 카메라로 바뀝니다. 오른쪽 위에 실험 모드 로고도 표시됩니다."));

  // experimental mode and openpilot longitudinal (alpha) are set from the OPTECH > 고급 tab
  // (apilot switches acc / blended on its own; experimental mode would force blended all the time)
  const bool show_advanced = false;
  e2e_toggle->setVisible(show_advanced);

  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (!CP.getExperimentalLongitudinalAvailable()) {
      params.remove("ExperimentalLongitudinalEnabled");
    }
    op_long_toggle->setVisible(show_advanced && CP.getExperimentalLongitudinalAvailable());

    const bool op_long = CP.getOpenpilotLongitudinalControl() && !CP.getExperimentalLongitudinalAvailable();
    const bool exp_long_enabled = CP.getExperimentalLongitudinalAvailable() && params.getBool("ExperimentalLongitudinalEnabled");
    if (op_long || exp_long_enabled) {
      // normal description and toggle
      e2e_toggle->setEnabled(true);
      e2e_toggle->setDescription(e2e_description);
    } else {
      // no long for now
      e2e_toggle->setEnabled(false);
      params.remove("ExperimentalMode");

      const QString no_long = tr("이 차는 순정 ACC로 가감속을 제어하므로 지금은 실험 모드를 쓸 수 없습니다.");
      const QString exp_long = tr("실험 모드를 쓰려면 openpilot 가감속 제어(알파)를 켜세요.");
      e2e_toggle->setDescription("<b>" + (CP.getExperimentalLongitudinalAvailable() ? exp_long : no_long) + "</b><br><br>" + e2e_description);
    }

    e2e_toggle->refresh();
  } else {
    e2e_toggle->setDescription(e2e_description);
    op_long_toggle->setVisible(show_advanced);
  }
}

// Power Off button (Soul EV tester, 2026-11-24): upstream shows it only offroad, but with the harness the car on in P
// (parking brake on) still counts as onroad, so the screen had no way to turn the device off. Onroad it now shows when
// the car is in P and openpilot is off. Once the device is off the panda gets no heartbeat, goes to its idle mode
// after ~5 s and passes the stock camera's messages to the car again (safety_defaults.h nooutput forwarding).
static bool optech_poweroff_ok(const UIState &s) {
  if (!s.scene.started) return true;
  if (s.engaged()) return false;
  return s.sm->rcv_frame("carState") > s.scene.started_frame &&
         (*s.sm)["carState"].getCarState().getGearShifter() == cereal::CarState::GearShifter::PARK;
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl(tr("동글 ID"), getDongleId().value_or(tr("N/A"))));
  addItem(new LabelControl(tr("시리얼"), params.get("HardwareSerial").c_str()));

  QHBoxLayout *reset_layout = new QHBoxLayout();
  reset_layout->setSpacing(30);

  // reset calibration button
  QPushButton *restart_openpilot_btn = new QPushButton(tr("소프트 재시작"));
  restart_openpilot_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(restart_openpilot_btn);
  QObject::connect(restart_openpilot_btn, &QPushButton::released, [=]() {
    emit closeSettings();
    QTimer::singleShot(1000, []() {
      Params().putBool("SoftRestartTriggered", true);
    });
  });

  // reset calibration button
  QPushButton *reset_calib_btn = new QPushButton(tr("Reset Calibration"));
  reset_calib_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(reset_calib_btn);
  QObject::connect(reset_calib_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm(tr("캘리브레이션과 학습 값을 초기화할까요?"), tr("Reset"), this)) {
      Params().remove("CalibrationParams");
      Params().remove("LiveParameters");
      emit closeSettings();
      QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    }
  });

  addItem(reset_layout);

  // offroad-only buttons

  auto dcamBtn = new ButtonControl(tr("Driver Camera"), tr("PREVIEW"),
                                   tr("Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"));
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl(tr("Reset Calibration"), tr("RESET"), "");
  connect(resetCalibBtn, &ButtonControl::showDescriptionEvent, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("Reset"), this)) {
      params.remove("CalibrationParams");
      params.remove("LiveTorqueParameters");
    }
  });
  addItem(resetCalibBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl(tr("Review Training Guide"), tr("REVIEW"), tr("Review the rules, features, and limitations of openpilot"));
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm(tr("Are you sure you want to review the training guide?"), tr("Review"), this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl(tr("Regulatory"), tr("VIEW"), "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      ConfirmationDialog::rich(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  auto translateBtn = new ButtonControl(tr("Change Language"), tr("CHANGE"), "");
  connect(translateBtn, &ButtonControl::clicked, [=]() {
    QMap<QString, QString> langs = getSupportedLanguages();
    QString selection = MultiOptionDialog::getSelection(tr("Select a language"), langs.keys(), langs.key(uiState()->language), this);
    if (!selection.isEmpty()) {
      // put language setting, exit Qt UI, and trigger fast restart
      Params().put("LanguageSetting", langs[selection].toStdString());
      qApp->exit(18);
      watchdog_kick();
    }
  });
  addItem(translateBtn);

  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    for (auto btn : findChildren<ButtonControl *>()) {
        //btn->setEnabled(offroad);
        btn->setEnabled(true);
    }
  });

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton(tr("Reboot"));
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *rebuild_btn = new QPushButton(tr("다시 빌드"));
  rebuild_btn->setObjectName("rebuild_btn");
  power_layout->addWidget(rebuild_btn);
  QObject::connect(rebuild_btn, &QPushButton::clicked, [=]() {

    if (ConfirmationDialog::confirm(tr("다시 빌드할까요?"), tr("Reset"), this)) {
      std::system("cd /data/openpilot && scons -c");
      std::system("rm /data/openpilot/.sconsign.dblite");
      std::system("rm /data/openpilot/prebuilt");
      std::system("rm -rf /tmp/scons_cache");
      if (Hardware::TICI())
        std::system("sudo reboot");
      else
        std::system("reboot");
    }
  });

  QPushButton *poweroff_btn = new QPushButton(tr("Power Off"));
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (!Hardware::PC()) {
    // was: visible only offroad (offroadTransition)
    connect(uiState(), &UIState::uiUpdate, poweroff_btn, [=](const UIState &s) {
      const bool ok = optech_poweroff_ok(s);
      if (poweroff_btn->isVisibleTo(this) != ok) poweroff_btn->setVisible(ok);
    });
  }

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #rebuild_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #rebuild_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      tr("openpilot requires the device to be mounted within 4° left or right and "
         "within 5° up or 8° down. openpilot is continuously calibrating, resetting is rarely required.");
  std::string calib_bytes = Params().get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != 0) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += tr(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? tr("down") : tr("up"),
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? tr("left") : tr("right"));
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reboot?"), tr("Reboot"), this)) {
      // Check engaged again in case it changed while the dialog was open
      if (!uiState()->engaged()) {
        Params().putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert(tr("Disengage to Reboot"), this);
  }
}

void DevicePanel::poweroff() {
  if (uiState()->engaged()) {
    ConfirmationDialog::alert(tr("Disengage to Power Off"), this);
  } else if (!optech_poweroff_ok(*uiState())) {
    ConfirmationDialog::alert("시동이 켜져 있을 때는 기어를 P에 놓아야 끌 수 있어요.", this);
  } else if (ConfirmationDialog::confirm(tr("Are you sure you want to power off?"), tr("Power Off"), this)) {
    // check again in case it changed while the dialog was open (engaged, or out of P with the car on)
    if (optech_poweroff_ok(*uiState())) {
      Params().putBool("DoShutdown", true);
    }
  }
}

C2NetworkPanel::C2NetworkPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);

  ListWidget *list = new ListWidget();
  list->setSpacing(30);
  // wifi + tethering buttons
#ifdef QCOM
  auto wifiBtn = new ButtonControl("와이파이 설정", "열기");
  QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
  list->addItem(wifiBtn);

  auto tetheringBtn = new ButtonControl("테더링 설정", "열기");
  QObject::connect(tetheringBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_tethering(); });
  list->addItem(tetheringBtn);
#endif
  ipaddress = new LabelControl("IP 주소", "");
  list->addItem(ipaddress);

  // SSH key management
  list->addItem(new SshToggle());
  list->addItem(new SshControl());
  layout->addWidget(list);
  layout->addStretch(1);
}

void C2NetworkPanel::showEvent(QShowEvent *event) {
  ipaddress->setText(getIPAddress());
}

QString C2NetworkPanel::getIPAddress() {
  std::string result = util::check_output("ifconfig wlan0");
  if (result.empty()) return "";

  const std::string inetaddrr = "inet addr:";
  std::string::size_type begin = result.find(inetaddrr);
  if (begin == std::string::npos) return "";

  begin += inetaddrr.length();
  std::string::size_type end = result.find(' ', begin);
  if (end == std::string::npos) return "";

  return result.substr(begin, end - begin).c_str();
}

QWidget *network_panel(QWidget *parent) {
#ifdef QCOM
  return new C2NetworkPanel(parent);
#else
  return new Networking(parent);
#endif
}

// SupportedCars lines are "CAR|brand|display" (selfdrive/car/hyundai/car_menu.py); an old plain "CAR" line also works
static QStringList get_list_utf8(const char* path) {
  QStringList stringList;
  QFile textFile(path);
  if (textFile.open(QIODevice::ReadOnly)) {
    QTextStream textStream(&textFile);
    textStream.setCodec("UTF-8");
    while (true) {
      QString line = textStream.readLine();
      if (line.isNull()) break;
      if (!line.trimmed().isEmpty()) stringList.append(line);
    }
  }
  return stringList;
}

// the menu name of a CAR string (the CAR string itself, kept in the list file for later use)
static QString car_menu_name(const QString &car) {
  // the menu file has the name without the brand ("K5 DL3 2020-2023"); the car button shows "KIA K5 DL3 2020-2023"
  if (car.isEmpty()) return QString();
  for (const QString &line : get_list_utf8("/data/params/d/SupportedCars")) {
    const QStringList parts = line.split('|');
    if (parts.size() >= 3 && parts[0] == car) {
      return parts[2].startsWith(parts[1] + " ") ? parts[2] : parts[1] + " " + parts[2];
    }
  }
  return car;
}

void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
  if (!param.isEmpty()) {
    emit expandToggleDescription(param);
  }
}

// Layout follows carrot (C3X) c3-wip selfdrive/ui/qt/offroad/settings.cc:
// round "×" close button, Device / Network / Toggles / Software / CarrotPilot
SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  panel_widget = new QStackedWidget();

  // close button
  QPushButton *close_btn = new QPushButton("×");
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 110px;
      padding-bottom: 16px;
      border-radius: 80px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(160, 160);
  sidebar_layout->addSpacing(30);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);
  QObject::connect(device, &DevicePanel::closeSettings, this, &SettingsWindow::closeSettings);
  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  // index 2 = Toggles (opened directly by the experimental mode button)
  QList<QPair<QString, QWidget *>> panels = {
    {tr("Device"), device},
#ifdef QCOM
    {tr("Network"), new C2NetworkPanel(this)},
#else
    {tr("Network"), new Networking(this)},
#endif
    {tr("Toggles"), toggles},
    {tr("Software"), new SoftwarePanel(this)},
    {"OPTECH", new CarrotPanel(this)},
  };

#ifdef ENABLE_MAPS
  //auto map_panel = new MapPanel(this);
  //panels.push_back({tr("Navigation"), map_panel});
  //QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 58px;
        font-weight: 500;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )");
    btn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    // Network panel handles its own margins; the carrot panel is wide, so keep its sides tight
    const int lr_margin = name == tr("Network") ? 0 : (name == "OPTECH" ? 15 : 50);
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(20, 40, 40, 40);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(360);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
    QStackedWidget, ScrollView {
      background-color: #292929;
      border-radius: 30px;
    }
  )");
}



/////////////////////////////////////////////////////////////////////////

// CarrotPilot panel: tab row Start / Cruise / Speed / Tuning / Disp / Path (selected tab green),
// same as carrot c3-wip CarrotPanel. The items are this C2 branch's own apilot/carrot params,
// sorted into carrot's tabs.
// carrot draws these rows without a leading icon
static const char *ROAD_ICON = "";
static const char *SHELL_ICON = "";

// optech '주행 기록 내려받기': the device's addresses on Wi-Fi / hotspot, as the log page URL (selfdrive/carrot/tools/
// logserver.py, port 8088). The phone hotspot / the device's own hotspot (192.168.43.x) first
static QStringList log_page_urls() {
  QStringList urls, others;
  for (const QNetworkInterface &nif : QNetworkInterface::allInterfaces()) {
    if (!(nif.flags() & QNetworkInterface::IsUp) || (nif.flags() & QNetworkInterface::IsLoopBack)) continue;
    if (nif.name().startsWith("rmnet")) continue;  // mobile data: a phone can't reach it
    for (const QNetworkAddressEntry &e : nif.addressEntries()) {
      if (e.ip().protocol() != QAbstractSocket::IPv4Protocol) continue;
      const QString url = QString("http://%1:8088").arg(e.ip().toString());
      (e.ip().toString().startsWith("192.168.43.") ? urls : others) << url;
    }
  }
  return urls + others;
}

// QR code + how to use it, for the phone to open the log page
class LogQrDialog : public QDialogBase {
public:
  LogQrDialog(QWidget *parent, const QStringList &urls) : QDialogBase(parent) {
    QHBoxLayout *main = new QHBoxLayout(this);
    main->setContentsMargins(60, 50, 60, 50);
    main->setSpacing(60);

    QLabel *qr = new QLabel();
    qrcodegen::QrCode code = qrcodegen::QrCode::encodeText(urls[0].toUtf8().data(), qrcodegen::QrCode::Ecc::MEDIUM);
    const int sz = code.getSize();
    QImage im(sz + 8, sz + 8, QImage::Format_RGB32);  // 4-module white border
    im.fill(qRgb(255, 255, 255));
    for (int y = 0; y < sz; y++) {
      for (int x = 0; x < sz; x++) {
        if (code.getModule(x, y)) im.setPixel(x + 4, y + 4, qRgb(0, 0, 0));
      }
    }
    const int px = (720 / (sz + 8)) * (sz + 8);
    qr->setPixmap(QPixmap::fromImage(im.scaled(px, px, Qt::KeepAspectRatio, Qt::FastTransformation)));
    main->addWidget(qr, 0, Qt::AlignVCenter);

    QVBoxLayout *right = new QVBoxLayout();
    right->setSpacing(30);
    QLabel *title = new QLabel("폰 카메라로 QR 코드를 찍으세요");
    title->setStyleSheet("font-size: 60px; font-weight: 600;");
    title->setWordWrap(true);
    right->addWidget(title);
    QLabel *desc = new QLabel(
      "열린 화면에서 '전체 기록 한 번에 받기'를 누르고, 받은 파일을 카카오톡으로 보내 주세요.\n\n"
      "폰이 이 기기와 같은 와이파이나 핫스팟에 연결돼 있어야 열립니다. 15분 뒤 자동으로 닫힙니다.\n\n"
      "주소: " + urls.join("  /  "));
    desc->setStyleSheet("font-size: 40px; color: #CCCCCC;");
    desc->setWordWrap(true);
    right->addWidget(desc);
    right->addStretch(1);
    QPushButton *close = new QPushButton("닫기");
    close->setFixedHeight(140);
    close->setStyleSheet("font-size: 50px; border-radius: 20px; background-color: #393939;");
    QObject::connect(close, &QPushButton::clicked, this, &QDialog::accept);
    right->addWidget(close);
    main->addLayout(right, 1);
  }
};

// group title inside a tab list
static QLabel *carrot_section(const QString &title) {
  QLabel *label = new QLabel(title);
  label->setStyleSheet("font-size: 42px; font-weight: 600; color: #33ab4c; padding-top: 25px; padding-left: 10px;");
  return label;
}

CarrotPanel::CarrotPanel(QWidget* parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  homeScreen = new QWidget(this);
  carrotLayout = new QVBoxLayout(homeScreen);
  carrotLayout->setMargin(0);

  QHBoxLayout* select_layout = new QHBoxLayout();
  select_layout->setSpacing(10);

  const QStringList tab_names = {"시작", "주행", "속도", "조향", "화면", "고급"};
  for (int i = 0; i < tab_names.size(); i++) {
    QPushButton* tab = new QPushButton(tab_names[i]);
    tab->setObjectName(QString("tab_btn_%1").arg(i));
    QObject::connect(tab, &QPushButton::clicked, this, [this, i]() {
      this->currentCarrotIndex = i;
      this->togglesCarrot(i);
      updateButtonStyles();
    });
    select_layout->addWidget(tab);
  }
  updateButtonStyles();
  carrotLayout->addLayout(select_layout, 0);

  QWidget* toggles = new QWidget();
  QVBoxLayout* toggles_layout = new QVBoxLayout(toggles);

  // ---- 시작: car ----
  startToggles = new ListWidget(this);
  QString selected = QString::fromStdString(Params().get("SelectedCar"));
  QPushButton* selectCarBtn = new QPushButton(selected.length() ? car_menu_name(selected) : tr("차종 선택"));
  selectCarBtn->setObjectName("selectCarBtn");
  selectCarBtn->setStyleSheet(R"(
    QPushButton {
      margin-top: 20px; margin-bottom: 20px; padding: 10px; height: 120px; border-radius: 15px;
      color: #FFFFFF; background-color: #2C2CE2;
    }
    QPushButton:pressed {
      background-color: #2424FF;
    }
  )");
  selectCar = new SelectCar(this);
  connect(selectCarBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(selectCar); });
  connect(selectCar, &SelectCar::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  // '레이 EV 설정' / '포터 II EV 설정': shown open right under the car button while that car is the
  // chosen one; choosing such a car also pops up a note to check them
  for (int i = 0; i < CcOnlyCarPage::count(); i++) {
    CcOnlyCarPage* page = new CcOnlyCarPage(i, this);
    page->setVisible(selected == CcOnlyCarPage::carName(i));
    ccOnlyPages.append(page);
  }
  connect(selectCar, &SelectCar::selectedCar, [=]() {
    QString car = QString::fromStdString(Params().get("SelectedCar"));
    selectCarBtn->setText(car.length() ? car_menu_name(car) : tr("차종 선택"));
    QString title;
    for (int i = 0; i < ccOnlyPages.size(); i++) {
      const bool mine = car == CcOnlyCarPage::carName(i);
      ccOnlyPages[i]->setVisible(mine);
      if (mine) title = CcOnlyCarPage::menuTitle(i);
    }
    main_layout->setCurrentWidget(homeScreen);
    if (!title.isEmpty()) {
      ConfirmationDialog::alert("이 차량은 따로 확인할 설정이 있어요.\n차종 버튼 아래의 '" + title + "'을 확인해 주세요.", this);
    }
  });
  startToggles->addItem(selectCarBtn);
  for (auto page : ccOnlyPages) startToggles->addItem(page);
  // (no 'GPS 확인' button on 고급 since 2026-11-26, user; boardd still powers the uBlox while GpsCheck is set)
  // '사용 방식' (EngageWithLong: 조향+가감속 / 조향만) was removed 2026-11-27 for carrot's cruise button rule (controlsd
  // engage_long_user, cruise_helper button_control / stock_scc_follow); this note at the top of 주행 says how the buttons work
  QLabel *button_note = new QLabel("● 크루즈 버튼: 순정 SCC 차는 누르면 조향이 켜지고, 다시 누르면 모두 꺼집니다. 롱컨 차는 SET이나 RES로 켭니다.\n"
                                   "● SET(-) / RES(+): 가감속을 켭니다. SET은 지금 속도로, RES는 이전 설정 속도로 켜지고, 켜져 있을 때는 설정 속도를 바꿉니다.\n"
                                   "● CANCEL: 짧게 누르면 가감속만 꺼지고 조향은 남습니다 (SET/RES로 다시 켜기). 0.4초 넘게 누르면 모두 꺼집니다.\n"
                                   "● 브레이크: 가감속만 꺼지고, 떼면 아래 '브레이크 후 다시 켜기'를 따릅니다.");
  button_note->setWordWrap(true);
  button_note->setStyleSheet("font-size: 36px; color: #E0E0E0; background-color: #333333; border-radius: 10px; padding: 20px; margin: 10px 0px;");

  // ---- 주행: brake / cruise buttons ----
  cruiseToggles = new ListWidget(this);
  cruiseToggles->addItem(button_note);  // how the cruise buttons work (replaces the '사용 방식' choice)
  cruiseToggles->addItem(carrot_section("브레이크"));
  // OptechBrakeResumeMode (cruise_helper): what speed control does after the driver brakes and lets go (조향+가감속)
  auto brake_resume_btn = new ButtonControl("", "선택",
    "조향+가감속을 쓰는 중에 브레이크를 밟았다 뗐을 때 가감속을 어떻게 다시 켤지 고릅니다.\n"
    "현재 속도로 (처음 값): 브레이크로 줄인 그 속도로 다시 켜집니다 (시속 30km 미만에서 앞차를 따라갈 때는 원래 설정 속도). "
    "브레이크로 속도를 줄이면 줄인 속도가 유지됩니다.\n"
    "원래 설정 속도로: 브레이크를 밟기 전의 설정 속도로 다시 가속합니다.\n"
    "다시 켜지 않음: 순정 크루즈처럼 꺼진 채로 있고, RES(+)나 SET(-)을 눌러야 다시 켜집니다. "
    "'스마트 크루즈 컨트롤 작동조건이 아님' 메시지가 자주 뜨면 이것을 고르세요.");
  const QStringList brake_resume_names = {"원래 설정 속도로", "현재 속도로", "다시 켜지 않음"};
  auto brake_resume_name = [=]() {
    int v = std::atoi(Params().get("OptechBrakeResumeMode").c_str());
    if (Params().get("OptechBrakeResumeMode").empty() || v < 0 || v > 2) v = 1;
    return "브레이크 후 다시 켜기: " + brake_resume_names[v];
  };
  brake_resume_btn->setTitle(brake_resume_name());
  QObject::connect(brake_resume_btn, &ButtonControl::clicked, [=]() {
    const QString cur = brake_resume_name().section(": ", 1);
    const QString sel = MultiOptionDialog::getSelection("브레이크를 밟았다 뗐을 때", brake_resume_names, cur, this);
    if (sel.isEmpty()) return;
    Params().put("OptechBrakeResumeMode", std::to_string(brake_resume_names.indexOf(sel)));
    brake_resume_btn->setTitle(brake_resume_name());
  });
  cruiseToggles->addItem(brake_resume_btn);
  // OptechGasResume (cruise_helper): brake, then straight onto the gas also brings speed control back (driver feedback)
  cruiseToggles->addItem(new ParamControl("OptechGasResume", "악셀로 다시 켜기",
    "켜 두면(초록색) 브레이크로 꺼진 가감속이, 악셀을 밟고 있을 때도 다시 켜집니다. "
    "시속 30km 이상에서 악셀을 0.5초 넘게 밟고 있으면, 그때 속도로 켜집니다. "
    "악셀을 더 밟아 빨라지면 설정 속도도 따라 올라갑니다 (속도 탭 '가속페달 속도 맞춤 상한'까지). "
    "핸들을 많이 꺾었거나 빨간 신호 앞에서는 켜지지 않고, 크루즈 버튼으로 직접 끈 뒤에도 켜지지 않습니다.",
    ROAD_ICON, this));
  cruiseToggles->addItem(choice("SoftHoldMode", "소프트홀드",
    "정차 후 브레이크를 떼도 차를 멈춘 상태로 잡아 주는 기능입니다.\n"
    "사용 안 함(처음 값) / 사용 / SCC와 함께 사용 (일부 차량은 전자식 주차 브레이크가 걸릴 수 있음)",
    {"사용 안 함", "사용", "SCC와 함께 사용"}));
  cruiseToggles->addItem(carrot_section("크루즈 버튼"));
  cruiseToggles->addItem(choice("LongControlActiveSound", "크루즈 알림음", "크루즈가 켜지고 꺼질 때 나는 알림음 크기입니다. 처음 값: 작게",
    {"끄기", "작게", "크게"}, 1));
  cruiseToggles->addItem(new CValueControl("CruiseSpeedUnit", "버튼 한 번에 바뀌는 속도", "크루즈 속도 버튼을 한 번 누를 때 바뀌는 속도(km/h)입니다.", ROAD_ICON, 1, 20, 1));
  // optech: same camera items as optech-spst - 내장 카메라 DB / 카메라 감속 (on/off) / 내비 연동 방식 (hkg only).
  // 내비 연동 방식 (OptechNaviLinkMode, user 2026-11-25) names the phone app: 0 사용 안 함, 1 TmapNda, 2 당근내비 (it was
  // 0 / 1 APM / 2 APM + 순정 / 3 NDA; manager.py moves old values at boot). AutoNaviSpeedCtrl stays the internal switch
  // road_speed_limiter / cruise_helper read: 0 camera slowdown off, 1 no app, 3 NDA + carrot protocol + the car's navi
  // (TmapNda speaks both), 2 carrot protocol + the car's navi (당근내비; APM is no longer received). The toggle writes 0
  // or the mode's value.
  // The bump speed item was removed (bump data only comes from the navi app; 35 km/h stays).
  auto navi_ctrl = [](int mode) { return mode == 1 ? 3 : mode == 2 ? 2 : 1; };
  cruiseToggles->addItem(carrot_section("과속카메라"));
  {
    Params cam_params;
    const int cam_ctrl = std::atoi(cam_params.get("AutoNaviSpeedCtrl").c_str());
    int cam_mode = std::atoi(cam_params.get("OptechNaviLinkMode").c_str());
    if (cam_params.get("OptechNaviLinkMode").empty() || cam_mode < 0 || cam_mode > 2) {
      cam_mode = 1;
      cam_params.put("OptechNaviLinkMode", "1");
    }
    if (cam_ctrl > 0 && cam_ctrl != navi_ctrl(cam_mode)) cam_params.put("AutoNaviSpeedCtrl", std::to_string(navi_ctrl(cam_mode)));
  }
  ToggleControl *cam_decel = new ToggleControl("카메라 감속", "과속카메라 앞에서 제한 속도까지 미리 줄입니다. 켜면 아래에 동작 설명이 나옵니다.", ROAD_ICON,
                                               std::atoi(Params().get("AutoNaviSpeedCtrl").c_str()) > 0, this);
  // shown right under '카메라 감속' while it is on: how it works with the navigation link and with the built-in DB
  QLabel *cam_note = new QLabel("● 과속카메라 앞 '감속 시작'에 정한 시간(처음 값 22초) 전부터 설정 속도를 조금씩 내려서, '감속 끝'(6초 전)까지 '목표 속도'(처음 값: 제한 속도 그대로 100%)에 맞춥니다. "
                                "구간단속은 구간이 끝날 때까지 제한 속도를 지킵니다.\n"
                                "● 내비 연동(TmapNda·당근내비): 폰 내비 앱이 보내는 카메라 정보를 먼저 쓰고, 앱 정보가 없을 때는 내장 카메라 DB와 차 순정 내비의 카메라 정보로 채웁니다.\n"
                                "● 내장 카메라 DB만(내비 연동 '사용 안 함'): 내비 앱 없이 기기 GPS와 전국 카메라 위치(약 3만 1천 대)로 찾습니다. "
                                "이동식·새 카메라는 모르고, 반대 차로의 카메라에도 줄일 수 있습니다.\n"
                                "● 가감속이 켜져 있을 때만 속도를 줄입니다. GPS가 안 잡히면 동작하지 않습니다.");
  cam_note->setWordWrap(true);
  cam_note->setStyleSheet("font-size: 36px; color: #E0E0E0; background-color: #333333; border-radius: 10px; padding: 20px; margin: 10px 0px;");
  // AutoNaviSpeedCtrlStart / End / SafetyFactor (cruise_helper reads them while driving: no reboot needed)
  QList<QWidget*> cam_tune = {
    new CValueControl("AutoNaviSpeedCtrlStart", "감속 시작 (초 전)", "카메라에 도착하기 몇 초 전부터 설정 속도를 줄이기 시작할지 정합니다. 클수록 일찍, 부드럽게 줄입니다. "
                      "시속 100km에서 22초면 약 600m 앞입니다. 처음 값 22.", ROAD_ICON, 5, 60, 1),
    new CValueControl("AutoNaviSpeedCtrlEnd", "감속 끝 (초 전)", "카메라에 도착하기 몇 초 전까지 목표 속도에 맞출지 정합니다. 클수록 카메라보다 일찍 목표 속도가 됩니다. "
                      "처음 값 6.", ROAD_ICON, 0, 20, 1),
    new CValueControl("AutoNaviSpeedSafetyFactor", "목표 속도 (제한 속도의 %)", "카메라 앞과 구간단속 안에서 제한 속도의 몇 %로 달릴지 정합니다. "
                      "100이면 제한 속도 그대로(단속 100km/h → 100km/h), 105면 5% 더 빠르게 (100km/h → 105km/h). "
                      "단속 기준을 넘지 않게 주의하세요. 처음 값 100.", ROAD_ICON, 90, 120, 1),
  };
  // 1. 과속 단속 알림  2. 내비 연동 방식  3. 내장 카메라 DB: with the note and the tuning values they are shown only
  // while '카메라 감속' is on; when it is off only '카메라 감속' is left in this section
  // carcontroller: while the camera slowdown starts (activeHda 2) the value goes into LKAS11 CF_Lkas_LdwsLH/RHWarning
  // for 0-3 s and 5-8 s, i.e. the car's own lane departure warning (K7 HEV tester 2026-11-24: beeps + cluster warning
  // surprised him; the old text promised '1 핸들 진동, 2 계기판 표시, 3 HUD 표시', but every value 1-3 does the same)
  // the value goes into CF_Lkas_LdwsLH/RHWarning as it is: openpilot's own lane departure warning uses 1 on the
  // Genesis G80 / G90 and 2 on the other cars (carcontroller process_hud_alert)
  ButtonControl *cam_haptic = choice("HapticFeedbackWhenSpeedCamera", "과속 단속 알림",
    "카메라 앞에서 속도를 줄이기 시작할 때 차의 차선 이탈 경고를 빌려 알려 줍니다 (3초씩 두 번).\n"
    "사용 안 함(처음 값): 알리지 않고 속도만 줄입니다.\n"
    "차선 경고 1·2·3: 계기판에 차선 이탈 경고가 뜨고 경고음이 납니다. 핸들 진동이 있는 차는 핸들도 떨립니다. "
    "차마다 맞는 번호가 다릅니다: 제네시스 G80·G90은 1, 다른 현대·기아는 보통 2입니다 (openpilot이 차선 이탈 경고에 쓰는 값). "
    "경고가 안 뜨거나 모양이 이상하면 다른 번호를 골라 보세요.",
    {"사용 안 함", "차선 경고 1", "차선 경고 2", "차선 경고 3"});
  ButtonControl *navi_mode = choice("OptechNaviLinkMode", "내비 연동 방식",
    "폰 내비 앱에서 카메라·과속방지턱·길안내 정보를 받습니다. 쓰는 앱에 맞춰 고르세요.\n"
    "사용 안 함: 내비 앱 정보를 받지 않고 내장 카메라 DB만 씁니다.\n"
    "TmapNda(처음 값): 앱의 오픈파일럿 연결이 당근 방식이든 NDA 방식이든 모두 받습니다. 앱에서 NDA 방식으로 두면 주행 화면에 TMAP으로 떠서 당근내비(CARROT)와 구별됩니다.\n"
    "당근내비: 당근 방식으로 받습니다.\n"
    "TmapNda·당근내비 공통: 폰과 C2가 같은 와이파이(핫스팟)에 붙어 있어야 합니다. 앱 정보가 없을 때는 내장 카메라 DB와 차 순정 내비의 카메라 정보로 채웁니다.\n"
    "'카메라 감속'이 켜져 있을 때 적용되고, 바꾼 뒤 재부팅해야 적용됩니다.",
    {"사용 안 함", "TmapNda", "당근내비"}, 1, [=](int cam_v) {
      Params cam_params;
      if (std::atoi(cam_params.get("AutoNaviSpeedCtrl").c_str()) > 0) cam_params.put("AutoNaviSpeedCtrl", std::to_string(navi_ctrl(cam_v)));
    });
  ParamControl *cam_db = new ParamControl("SpeedCamDB", "내장 카메라 DB", "내비 앱 없이 기기 GPS로 앞쪽 고정식 단속카메라(과속·신호과속·구간단속)를 찾습니다.\n전국 약 3만 1천 대(공공데이터)의 위치가 기기에 들어 있습니다. 구간단속은 구간이 끝날 때까지 제한속도를 지킵니다.\n\n● 함께 켜야 할 것: '카메라 감속' (끄면 화면에 CAM 표시만 나오고 속도는 줄이지 않습니다)\n● 안 되는 경우: 이동식 카메라, 새로 생긴 카메라 / GPS가 안 잡힐 때 / 스마트 크루즈가 없는 차(조향만 하는 차)\n● 주의: 반대 차로의 카메라에서도 속도를 줄일 수 있습니다. 내비 앱이 카메라 정보를 보내면 앱 정보를 먼저 씁니다.\n\n바꾼 뒤 재부팅해야 적용됩니다.", ROAD_ICON, this);
  QList<QWidget*> cam_group = {cam_note};
  cam_group += cam_tune;
  cam_group += QList<QWidget*>{cam_haptic, navi_mode, cam_db};
  QObject::connect(cam_decel, &ToggleControl::toggleFlipped, [=](bool cam_on) {
    Params cam_params;
    const int cam_mode = std::atoi(cam_params.get("OptechNaviLinkMode").c_str());
    cam_params.put("AutoNaviSpeedCtrl", cam_on ? std::to_string(navi_ctrl(cam_mode)) : "0");
    for (auto w : cam_group) w->setVisible(cam_on);
  });
  cruiseToggles->addItem(cam_decel);
  for (auto w : cam_group) cruiseToggles->addItem(w);
  // only after they are in the list: showing a widget that has no parent yet opens it as its own top-level window,
  // and the C2 (EGLFS, one window) crashes on that at start when 카메라 감속 is on
  const bool cam_shown = std::atoi(Params().get("AutoNaviSpeedCtrl").c_str()) > 0;
  for (auto w : cam_group) w->setVisible(cam_shown);

  // OptechSignalEnable '신호등 사용' (long_mpc -> controls/lib/optech_signal.py, StarPilot's stop light detector): the
  // HUD traffic light; OptechGreenAlert (cruise_helper: StarPilot's green light alert) is shown only while it is on
  cruiseToggles->addItem(carrot_section("신호등"));
  auto signal_on = new ParamControl("OptechSignalEnable", "신호등 사용",
    "켜 두면 주행 모델의 예측으로 빨간불·초록불을 판단해 주행 화면 가운데 위에 신호등을 보여 줍니다. "
    "차가 신호에 맞춰 서거나 출발하지는 않고, 화면 표시와 알림에만 씁니다.\n"
    "신호등을 직접 보는 것이 아니라서 노란불은 없고, 서지 않고 지나가는 초록불에서는 아무것도 켜지지 않습니다. "
    "끄면 신호등 표시와 초록불 알림이 모두 꺼집니다.", ROAD_ICON, this);
  cruiseToggles->addItem(signal_on);
  auto green_alert = new ParamControl("OptechGreenAlert", "초록불 알림",
    "빨간불에 서 있다가 초록불로 바뀌면 소리와 함께 '초록불로 바뀌었어요'를 띄워 줍니다. 오픈파일럿이 켜져 있을 때만 알려 줍니다.\n"
    "신호등을 직접 보는 것이 아니라, 주행 모델이 멈출지 출발할지 예측한 것으로 판단합니다. "
    "앞차가 출발할 때도 울릴 수 있고, 늦거나 빠를 수 있으니 출발 전에는 꼭 신호를 직접 확인하세요. "
    "주행 화면의 신호등 표시도 같은 판단으로 켜집니다.", ROAD_ICON, this);
  cruiseToggles->addItem(green_alert);
  // visibility only after addItem (a widget without a parent would open as its own window, see 카메라 감속 above)
  green_alert->setVisible(Params().getBool("OptechSignalEnable"));
  QObject::connect(signal_on, &ToggleControl::toggleFlipped, [=](bool on) { green_alert->setVisible(on); });

  // ---- 속도: cruise speed, camera / bump, accel ----
  speedToggles = new ListWidget(this);
  speedToggles->addItem(carrot_section("크루즈 속도"));
  speedToggles->addItem(new CValueControl("AutoSyncCruiseSpeedMax", "가속페달 속도 맞춤 상한", "크루즈(가감속)가 켜진 상태에서 악셀로 설정 속도보다 빨리 달리면, 설정 속도도 지금 속도로 따라 올라갑니다 (예: 설정 50km/h에서 악셀로 80km/h까지 가면 설정 속도 80km/h). 이 값(km/h)까지만 올립니다. 0:사용 안 함", ROAD_ICON, 0, 200, 10));
  speedToggles->addItem(new CValueControl("AutoSpeedUptoRoadSpeedLimit", "제한속도까지 자동 올림", "앞차를 따라갈 때 도로 제한속도의 이 비율(%)까지 설정 속도를 자동으로 올립니다. 0:사용 안 함, 100:제한속도까지", ROAD_ICON, 0, 200, 10));
  speedToggles->addItem(carrot_section("출발·정지 (출발이 굼뜨거나 정지가 거칠 때)"));
  speedToggles->addItem(new CValueControl("JerkStartLimit", "재출발 가속", "멈췄다가 다시 출발할 때 얼마나 빨리 속도를 붙일지 정합니다(0.1 단위). 높이면 빠르게, 낮추면 부드럽게 출발합니다. 출발 전에 기다리는 시간과는 상관없습니다.", ROAD_ICON, 1, 50, 1));
  speedToggles->addItem(new CValueControl("StartAccelApply", "출발 가속", "정차 후 출발할 때 처음 가속을 몇 % 더 줄지 정합니다. 출발이 굼뜨면 올려 보세요. 0:사용 안 함", ROAD_ICON, 0, 100, 10));
  speedToggles->addItem(new CValueControl("StopAccelApply", "정지 유지 힘", "멈춰 있을 때 브레이크를 얼마나 세게 잡을지 정합니다(%). 정차 중 차가 밀리면 올려 보세요. 0:사용 안 함", ROAD_ICON, 0, 100, 10));
  speedToggles->addItem(new CValueControl("StopDistance", "앞차 뒤 정지 거리", "앞차 뒤에 멈출 때의 거리(cm)입니다. 600이면 약 6m 뒤에 멈춥니다.", ROAD_ICON, 200, 1000, 50));
  speedToggles->addItem(carrot_section("속도별 가속 세기 (가속이 너무 세거나 약할 때)"));
  speedToggles->addItem(new CValueControl("CruiseMaxVals1", "가속: 0km/h", "이 속도에서 낼 수 있는 최대 가속 세기입니다(단위 0.01m/s²). 높이면 더 빠르게 가속하고, 낮추면 부드럽게 가속합니다.", ROAD_ICON, 1, 250, 5));
  speedToggles->addItem(new CValueControl("CruiseMaxVals2", "가속: 40km/h", "이 속도에서 낼 수 있는 최대 가속 세기입니다(단위 0.01m/s²). 높이면 더 빠르게 가속하고, 낮추면 부드럽게 가속합니다.", ROAD_ICON, 1, 250, 5));
  speedToggles->addItem(new CValueControl("CruiseMaxVals3", "가속: 60km/h", "이 속도에서 낼 수 있는 최대 가속 세기입니다(단위 0.01m/s²). 높이면 더 빠르게 가속하고, 낮추면 부드럽게 가속합니다.", ROAD_ICON, 1, 250, 5));
  speedToggles->addItem(new CValueControl("CruiseMaxVals4", "가속: 80km/h", "이 속도에서 낼 수 있는 최대 가속 세기입니다(단위 0.01m/s²). 높이면 더 빠르게 가속하고, 낮추면 부드럽게 가속합니다.", ROAD_ICON, 1, 250, 5));
  speedToggles->addItem(new CValueControl("CruiseMaxVals5", "가속: 110km/h", "이 속도에서 낼 수 있는 최대 가속 세기입니다(단위 0.01m/s²). 높이면 더 빠르게 가속하고, 낮추면 부드럽게 가속합니다.", ROAD_ICON, 1, 250, 5));
  speedToggles->addItem(new CValueControl("CruiseMaxVals6", "가속: 140km/h", "이 속도에서 낼 수 있는 최대 가속 세기입니다(단위 0.01m/s²). 높이면 더 빠르게 가속하고, 낮추면 부드럽게 가속합니다.", ROAD_ICON, 1, 250, 5));

  // ---- 조향: steering and curves ----
  steerToggles = new ListWidget(this);
  steerToggles->addItem(carrot_section("조향"));
  // OptechLatMethod '조향 방식' (hyundai/interface.py set_optech_lat_method, applied at the next start):
  // 0 토크 (default), 1 PID, 2 INDI, 3 LQR with OPKR's Hyundai/Kia defaults. Each method's values show only
  // when that method is chosen
  {
    const QStringList lm_names = {"토크 (기본)", "PID", "INDI", "LQR"};
    auto lm_mode = []() { return std::clamp(std::atoi(Params().get("OptechLatMethod").c_str()), 0, 3); };
    const QString lm_desc =
      "핸들을 얼마나 돌릴지 계산하는 방식을 고릅니다. 바꾼 뒤 기기를 다시 켜야 적용됩니다.\n"
      "\n"
      "[토크 (기본)] 지금 쓰는 방식입니다. 주행하며 차에 맞게 스스로 학습합니다. 잘 모르면 그대로 두세요.\n"
      "[PID] 오래된 기본 방식입니다. 핸들 각도 차이만큼 바로 돌립니다.\n"
      "[INDI] 핸들 움직임을 예측해서 돌립니다. 반응이 빠른 편입니다.\n"
      "[LQR] 핸들 각도와 움직임을 함께 보고 부드럽게 돌립니다.\n"
      "\n"
      "PID / INDI / LQR은 OPKR의 현대·기아 기본값으로 시작합니다. 차마다 맞는 값이 달라서 "
      "핸들이 흔들리거나 덜 꺾이면 아래 값을 조금씩 바꾸거나 토크로 돌아가세요.";
    auto lm_btn = new ButtonControl("", "선택", lm_desc);
    struct LmVal { int method; CValueControl* ctl; };
    std::vector<LmVal>* vals = new std::vector<LmVal>{
      {1, new CValueControl("OptechPidKp", "PID 비례 (Kp, 0.01 단위)", "핸들 각도 차이에 바로 반응하는 세기입니다. 높이면 빨리 꺾고, 너무 높으면 흔들립니다. 처음 값 25.", ROAD_ICON, 5, 100, 1)},
      {1, new CValueControl("OptechPidKi", "PID 적분 (Ki, 0.001 단위)", "오래 남은 차이를 조금씩 메우는 세기입니다. 커브에서 계속 덜 꺾이면 조금 올립니다. 처음 값 40.", ROAD_ICON, 0, 200, 1)},
      {1, new CValueControl("OptechPidKf", "PID 앞먹임 (Kf, 0.00001 단위)", "필요한 핸들 각도를 미리 넣어 주는 세기입니다. 처음 값 7.", ROAD_ICON, 0, 50, 1)},
      {2, new CValueControl("OptechIndiInner", "INDI 안쪽 게인 (0.1 단위)", "커브에 들어갈 때 꺾는 반응입니다. 처음 값 35.", ROAD_ICON, 10, 100, 1)},
      {2, new CValueControl("OptechIndiOuter", "INDI 바깥 게인 (0.1 단위)", "차로 가운데를 지키는 세기입니다. 처음 값 20.", ROAD_ICON, 10, 100, 1)},
      {2, new CValueControl("OptechIndiTime", "INDI 시간 상수 (0.1 단위)", "높이면 부드럽고 느리게, 낮추면 빠르게 움직입니다. 처음 값 14.", ROAD_ICON, 5, 50, 1)},
      {2, new CValueControl("OptechIndiAct", "INDI 조향 효율 (0.1 단위)", "낮추면 더 많이 꺾습니다. 너무 많이 꺾이면 올립니다. 처음 값 20.", ROAD_ICON, 5, 50, 1)},
      {3, new CValueControl("OptechLqrScale", "LQR 크기 (Scale)", "높이면 덜 꺾습니다. 처음 값 1500.", ROAD_ICON, 500, 3000, 50)},
      {3, new CValueControl("OptechLqrKi", "LQR 적분 (Ki, 0.001 단위)", "오래 남은 차이를 메우는 세기입니다. 처음 값 16.", ROAD_ICON, 0, 100, 1)},
      {3, new CValueControl("OptechLqrDcGain", "LQR DC 게인 (0.00001 단위)", "핸들 각도와 조향 힘의 비율입니다. 처음 값 265.", ROAD_ICON, 100, 500, 5)},
    };
    auto lm_refresh = [=]() {
      const int m = lm_mode();
      lm_btn->setTitle("조향 방식: " + lm_names[m]);
      for (auto& v : *vals) v.ctl->setVisible(v.method == m);
    };
    QObject::connect(lm_btn, &ButtonControl::clicked, [=]() {
      ConfirmationDialog info("<b>조향 방식</b><br><br>" + lm_desc.toHtmlEscaped().replace("\n", "<br>"), "선택하기", "취소", true, this);
      if (!info.exec()) return;
      const QString cur = lm_names[lm_mode()];
      const QString sel = MultiOptionDialog::getSelection("조향 방식", lm_names, cur, this);
      if (sel.isEmpty() || sel == cur) return;
      Params().put("OptechLatMethod", std::to_string(lm_names.indexOf(sel)));
      lm_refresh();
      ConfirmationDialog::alert("기기를 다시 켜야 적용됩니다.", this);
    });
    steerToggles->addItem(lm_btn);
    for (auto& v : *vals) steerToggles->addItem(v.ctl);
    lm_refresh();
  }
  steerToggles->addItem(new CValueControl("PathOffset", "경로 좌우 보정 (쏠릴 때)", "차가 차로 안에서 한쪽으로 치우쳐 달리면 보정합니다. -값:왼쪽으로, +값:오른쪽으로 옮깁니다.", ROAD_ICON, -200, 200, 1));
  steerToggles->addItem(new ParamControl("NNFF", "NNFF 조향 보정", "켜 두면(초록색) 차종별로 미리 학습된 작은 신경망으로 커브에서 필요한 핸들 힘을 더 정확히 계산합니다(twilsonco NNFF, 당근 c3-wip에서 이식). 현대·기아·제네시스 31개 차종에만 모델이 있고, 모델이 없는 차(K7, 그랜저 IG 등)는 켜도 기존 방식 그대로입니다. 토크 조향 차량에서만 동작합니다. 바꾼 뒤 재부팅해야 적용됩니다.", ROAD_ICON, this));
  steerToggles->addItem(new ParamControl("NNFFLite", "NNFF 라이트", "NNFF 모델이 없는 차(K7, 그랜저 등)용입니다. 켜 두면(초록색) 주행 모델이 예상한 앞쪽 커브 변화를 미리 보고, 커브에 들어가거나 나올 때 핸들 마찰 보정을 미리 넣어 늦게 꺾이거나 늦게 풀리는 느낌을 줄입니다. 보정 크기는 기존 마찰 값 안으로 제한됩니다. 모델이 있는 차에서 NNFF를 켜면 NNFF가 우선합니다. 토크 조향 차량에서만 동작합니다. 바꾼 뒤 재부팅해야 적용됩니다.", ROAD_ICON, this));
  steerToggles->addItem(new CValueControl("AutoLaneChangeSpeed", "차선변경 최소 속도", "방향지시등을 켰을 때 자동 차선 변경이 되는 최소 속도(km/h)입니다.", ROAD_ICON, 5, 60, 5));
  steerToggles->addItem(carrot_section("커브"));
  // cruise_helper: 2 = carrot vision curve speed, any other value > 0 = apilot road-design table (3 did the same as 1)
  if (Params().get("AutoCurveSpeedCtrlUse") == "3") Params().put("AutoCurveSpeedCtrlUse", "1");
  steerToggles->addItem(choice("AutoCurveSpeedCtrlUse", "커브 자동 감속",
    "커브를 만나면 미리 속도를 줄입니다.\n"
    "사용 안 함: 커브에서 따로 줄이지 않습니다.\n"
    "도로 설계 기준: 주행 경로가 휘어진 정도로 계산해서 줄입니다.\n"
    "당근 방식(처음 값): 카메라로 앞 커브를 보고 거리에 맞춰 미리 줄입니다.",
    {"사용 안 함", "도로 설계 기준", "당근 방식"}, 2));
  steerToggles->addItem(new CValueControl("AutoCurveSpeedFactor", "커브 감속 비율", "커브 감속 정도입니다(%). 높이면 커브에서 더 빨리 달리고, 낮추면 더 많이 감속합니다.", ROAD_ICON, 50, 300, 1));
  steerToggles->addItem(carrot_section("조향 세부 (커브에서 덜 꺾이거나 너무 꺾일 때)"));
  steerToggles->addItem(new CValueControl("SteerRatioApply", "조향비 고정", "핸들을 돌린 만큼 바퀴가 꺾이는 비율(조향비)을 고정합니다(0.1 단위, 150=15.0). 0:자동 학습값 사용", ROAD_ICON, 0, 300, 2));
  steerToggles->addItem(new CValueControl("LiveSteerRatioApply", "실시간 조향비 적용", "자동으로 학습한 조향비를 몇 %로 쓸지 정합니다. 커브에서 너무 많이 꺾이면(오버스티어) 낮추세요.", ROAD_ICON, 50, 110, 1));
  steerToggles->addItem(new CValueControl("SteerActuatorDelay", "조향 반응 지연 보정", "조향 명령을 보낸 뒤 실제로 핸들이 움직이기까지 걸리는 시간 보정입니다(0.01초 단위). 커브에서 늦게 꺾이면 조금 올려 보세요.", ROAD_ICON, 0, 100, 1));
  // 1 and 2 did the same while driving (latcontrol_torque reads the values every 0.3 s; 2 also set them at the start)
  if (Params().get("LateralTorqueCustom") == "2") Params().put("LateralTorqueCustom", "1");
  auto torqueCustom = new ParamControl("LateralTorqueCustom", "토크 사용자 설정", "켜 두면(초록색) 자동 학습을 끄고 아래 '토크 가속 계수'와 '토크 마찰 계수'를 씁니다 (주행 중 바꿔도 몇 초 안에 적용). 끄면(처음 값) 차량 기본값으로 시작해서 주행하며 자동으로 학습합니다.", ROAD_ICON, this);
  steerToggles->addItem(torqueCustom);
  auto torqueAccel = new CValueControl("LateralTorqueAccelFactor", "토크 가속 계수", "토크 조향에서 원하는 회전량을 내는 데 필요한 힘 계수입니다. 커브에서 덜 꺾이면 낮추고, 너무 꺾이면 높이세요. '토크 사용자 설정'이 켜져 있을 때만 적용됩니다.", ROAD_ICON, 1000, 4000, 10);
  steerToggles->addItem(torqueAccel);
  auto torqueFriction = new CValueControl("LateralTorqueFriction", "토크 마찰 계수", "핸들 마찰을 보정하는 값입니다. 직진에서 핸들이 늦게 반응하면 조금 올려 보세요. '토크 사용자 설정'이 켜져 있을 때만 적용됩니다.", ROAD_ICON, 0, 1000, 10);
  steerToggles->addItem(torqueFriction);
  // show the two torque values only while 토크 사용자 설정 is on
  auto showTorqueValues = [=](bool on) { torqueAccel->setVisible(on); torqueFriction->setVisible(on); };
  QObject::connect(torqueCustom, &ToggleControl::toggleFlipped, showTorqueValues);
  showTorqueValues(Params().getBool("LateralTorqueCustom"));

  // ---- 화면 ----
  dispToggles = new ListWidget(this);
  // (no '화면 표시 방식' menu: every device uses the optech 주행 화면, ShowHudMode 7, forced by manager.py; user 2026-11-23)
  // OptechSettingsTimeout '화면 유지 시간' (ui.cc Device::resetInteractiveTimout): how long the settings window stays
  // open and the screen stays on with the car off, untouched (replaces '화면 항상 켜기')
  {
    auto keep_btn = new ButtonControl("", "선택",
      "설정을 천천히 보고 만질 수 있게 시간 여유를 둡니다. "
      "설정한 시간이 지나면 자동으로 메인 화면으로 돌아갑니다. 차량에 연결되어 있으면 주행 화면으로 돌아갑니다. "
      "화면을 만지면 시간을 처음부터 다시 셉니다. 항상: 돌아가지 않습니다. "
      "차량에 연결하지 않고 켜 두었을 때는 메인 화면으로 돌아가면서 화면도 꺼지고, 화면을 누르면 다시 켜집니다. "
      "설정 화면을 보다가 주행이 시작되면 시간과 상관없이 바로 주행 화면으로 바뀝니다.");
    const QStringList keep_names = {"1분", "3분", "5분", "10분", "항상"};
    const QList<int> keep_secs = {60, 180, 300, 600, -1};  // -1: never (ui.cc updateWakefulness)
    auto keep_name = [=]() {
      const std::string v = Params().get("OptechSettingsTimeout");
      const int i = keep_secs.indexOf(v.empty() ? 180 : std::atoi(v.c_str()));
      return "화면 유지 시간: " + keep_names[i < 0 ? 1 : i];
    };
    keep_btn->setTitle(keep_name());
    QObject::connect(keep_btn, &ButtonControl::clicked, [=]() {
      const QString sel = MultiOptionDialog::getSelection("화면 유지 시간", keep_names, keep_name().section(": ", 1), this);
      if (sel.isEmpty()) return;
      Params().put("OptechSettingsTimeout", std::to_string(keep_secs[keep_names.indexOf(sel)]));
      keep_btn->setTitle(keep_name());
    });
    dispToggles->addItem(keep_btn);
  }
  // OptechDriveDim '주행 중 밝기' (ui.cc Device::updateBrightness): in D only (not N / P / R), 30 s without a touch
  // -> this %, -1 '화면 밝기 0%': backlight off (Soul EV tester 2026-11-26: screen fully off in D, bright again in P)
  {
    auto dim_btn = new ButtonControl("", "선택",
      "[주행 중 화면 어둡게] 고르면 기어가 D(주행)일 때만 해당됩니다. 화면을 30초 동안 만지지 않으면 고른 밝기로 어두워집니다. "
      "화면 밝기 0%를 고르면 화면이 완전히 검게 됩니다. "
      "화면을 터치하거나 기어를 N·P·R에 두면 화면이 다시 밝아지고, 다시 D로 바꾼 뒤 30초가 지나면 다시 적용됩니다. "
      "경고가 뜰 때도 바로 밝아집니다. 밤에 화면이 눈부실 때 쓰세요. "
      "항상 밝음(처음 값): 늘 원래 밝기 그대로입니다.");
    const QStringList dim_names = {"항상 밝음", "10%", "20%", "30%", "40%", "50%", "70%", "화면 밝기 0%"};
    const QList<int> dim_pcts = {0, 10, 20, 30, 40, 50, 70, -1};
    auto dim_name = [=]() {
      const int i = dim_pcts.indexOf(std::atoi(Params().get("OptechDriveDim").c_str()));
      return "주행 중 밝기: " + dim_names[i < 0 ? 0 : i];
    };
    dim_btn->setTitle(dim_name());
    QObject::connect(dim_btn, &ButtonControl::clicked, [=]() {
      const QString sel = MultiOptionDialog::getSelection("주행 중 밝기", dim_names, dim_name().section(": ", 1), this);
      if (sel.isEmpty()) return;
      const int pct = dim_pcts[dim_names.indexOf(sel)];
      Params().put("OptechDriveDim", std::to_string(pct));
      dim_btn->setTitle(dim_name());
      // user 2026-11-26: like the Bolt EV car choice, explain right after choosing how it brightens and comes back
      if (pct != 0) {
        const QString level = pct < 0 ? "화면 밝기 0%" : QString("%1%").arg(pct);
        const QString what = pct < 0 ? "화면이 검게 꺼집니다." : QString("화면이 %1% 밝기로 어두워집니다.").arg(pct);
        // rich (42 px): this text does not fit the 70 px alert, whose button then ends up off the screen.
        // 8 lines so it fits without scrolling
        ConfirmationDialog::rich("<b>주행 중 밝기: " + level + "</b><br>"
                                 "기어가 D일 때 화면을 30초 동안 만지지 않으면 " + what + "<br><br>"
                                 "<b>다시 밝아지는 때</b><br>· 화면을 터치할 때 · 기어를 N·P·R에 둘 때 · 경고가 뜰 때<br><br>"
                                 "<b>다시 " + level + "가 되는 때</b><br>"
                                 "· 터치한 뒤 30초 동안 안 만지면 · N·P·R에서 D로 바꾸고 30초가 지나면", this);
      }
    });
    dispToggles->addItem(dim_btn);
  }
  dispToggles->addItem(new ParamControl("OptechShowRecordBtn", "녹화 버튼",
    "켜 두면(초록색) 주행 화면 오른쪽 아래에 동그란 'REC' 녹화 버튼이 보입니다. 누르면 기기 화면을 동영상으로 녹화하고 "
    "(녹화 중에는 버튼이 빨개집니다), 다시 누르면 멈춥니다. 영상은 3분마다 나뉘어 기기 저장공간(videos 폴더)에 남습니다. "
    "주행 영상을 남기고 싶을 때만 켜세요. 끄면 버튼이 사라지고 하던 녹화도 멈춥니다.", SHELL_ICON, this));
  dispToggles->addItem(new ParamControl("OptechShowGpsSpeed", "GPS 속도",
    "켜 두면(초록색) 주행 화면 설정 속도 칸 오른쪽 작은 칸에 GPS로 잰 속도(km/h)를 초록 숫자로 보여 줍니다. 계기판 속도는 실제보다 몇 % 높게 "
    "나오므로, 실제로 몇 km/h로 달리는지 볼 때 씁니다. GPS가 안 잡히면 숫자 대신 초록 가로 막대가 나옵니다. 끄면 칸이 사라집니다.", SHELL_ICON, this));
  // (no '시간 표시' (ShowDateTime): the optech 주행 화면 has no clock, user 2026-11-23)
  // optech (user 2026-11-26): the 0 / 1 items are on/off switches like 녹화 버튼 (ui.cc still reads them with atoi)
  dispToggles->addItem(new ParamControl("ShowTpms", "타이어 공기압", "켜 두면(초록색) 주행 화면 오른쪽 맨 아래 칸에 네 바퀴의 타이어 공기압(psi)을 보여 줍니다. 위 줄이 앞바퀴(왼쪽·오른쪽), 아래 줄이 뒷바퀴입니다.", SHELL_ICON, this));
  dispToggles->addItem(new ParamControl("ShowGpsInfo", "GPS 상태", "켜 두면(초록색) 주행 화면 왼쪽 위(내비 연결 표시 옆)에 GPS가 잡혔는지와 신호 세기(막대 4칸)를 보여 줍니다. 초록: 잘 잡힘, 주황: 찾는 중·약함. GPS는 시동이 켜져 있을 때만 동작합니다.", SHELL_ICON, this));
  dispToggles->addItem(new ParamControl("ShowDeviceState", "기기 상태", "켜 두면(초록색) 주행 화면 오른쪽 가운데 칸에 기기의 CPU 온도와 CPU 사용률을 보여 줍니다. 80°C를 넘거나 사용률이 90%를 넘으면 빨갛게 깜빡입니다.", SHELL_ICON, this));
  dispToggles->addItem(new ParamControl("ShowBlindSpot", "후측방(BSD) 경고", "켜 두면(초록색) 옆 차로 뒤쪽에 차가 있을 때 도로 양옆(앞차 정보 옆)에 노란 차량 표시가 뜹니다. 그쪽으로 방향지시등을 켜면 그 표시가 깜빡입니다. 시속 30km 이상에서만 보이고, 차에 후측방 감지(BSD)가 있어야 합니다.", SHELL_ICON, this));
  // ShowRadarInfo '레이더 물체 표시' (paint.cc ui_draw_radar_info): 0 off, 1 moving objects + speed,
  // 2 also how far to the side, 3 also stopped objects (amber "정지")
  {
    auto radar_btn = new ButtonControl("", "선택",
      "앞 레이더가 잡은 주변 물체를 주행 화면에 네모로 보여 줍니다. 네모 안 숫자는 그 물체의 속도(km/h)입니다.\n"
      "초록: 같은 방향으로 가는 물체, 빨강: 마주 오는 물체, 주황 '정지': 서 있는 물체.\n"
      "사람·차·오토바이 같은 종류는 구별하지 못합니다. "
      "여러 물체를 보려면 차가 레이더 트랙을 지원해야 하고, 토글 화면의 '레이더 트랙 사용'을 켜야 합니다. "
      "지원하지 않는 차는 앞차 하나만 나옵니다.\n"
      "서 있는 물체까지 보이게 하면 가드레일·표지판도 잡혀서 화면이 복잡해질 수 있습니다.");
    const QStringList radar_names = {"안 보임", "움직이는 물체", "움직이는 물체 + 옆 거리", "전부 (서 있는 물체까지)"};
    auto radar_name = [=]() {
      const int v = std::atoi(Params().get("ShowRadarInfo").c_str());
      return "레이더 물체 표시: " + radar_names[std::clamp(v, 0, 3)];
    };
    radar_btn->setTitle(radar_name());
    QObject::connect(radar_btn, &ButtonControl::clicked, [=]() {
      const QString sel = MultiOptionDialog::getSelection("레이더 물체 표시", radar_names, radar_name().section(": ", 1), this);
      if (sel.isEmpty()) return;
      Params().put("ShowRadarInfo", std::to_string(radar_names.indexOf(sel)));
      radar_btn->setTitle(radar_name());
    });
    dispToggles->addItem(radar_btn);
  }
  dispToggles->addItem(new ParamControl("ShowConnInfo", "연결 상태 표시", "켜 두면(초록색) 주행 화면 왼쪽 위에 연결 상태를 보여 줍니다.\nTMAP: TmapNda가 NDA 방식으로 보내는 중\nCARROT: 당근 방식으로 들어옴 (당근내비, 당근 방식으로 설정한 TmapNda)\n초록은 정보가 들어오는 중, 회색은 연결됐지만 아직 정보가 없음\nHDA(보라): 차 순정 내비의 카메라 정보를 쓰는 중\nSCC2(빨강): SCC 버스2(롱컨) 차량\nRADAR(주황): 레이더 트랙 사용 중", SHELL_ICON, this));
  // '운전자 모니터링 아이콘' (ShowDmInfo 0 / 1); the mode itself is chosen on the 토글 page (optech_dm_mode)
  dmIcon = new ToggleControl("운전자 모니터링 아이콘",
    "주행 화면 오른쪽 아래(녹화 버튼을 켜 두면 그 왼쪽)에 운전자 모니터링 아이콘(얼굴 모양)을 보여 줍니다. 얼굴을 잘 보고 있으면 선명하게, 아니면 흐리게 보입니다. 토글 화면의 운전자 모니터링이 '기본 사용'일 때만 쓸 수 있습니다.",
    SHELL_ICON, std::atoi(Params().get("ShowDmInfo").c_str()) >= 1, this);
  QObject::connect(dmIcon, &ToggleControl::toggleFlipped, [=](bool on) {
    if (optech_dm_mode() == 0) Params().put("ShowDmInfo", on ? "1" : "0");
  });
  dispToggles->addItem(dmIcon);

  // ---- 고급: experimental / developer ----
  advancedToggles = new ListWidget(this);
  advancedToggles->addItem(carrot_section("기기"));
  // OptechOffShutdown (autoshutdownd): after a drive, turn the device off this long after the car is off (tester feedback)
  {
    auto off_btn = new ButtonControl("", "선택",
      "[기기 전원을 끄는 시간] 상시 전원(시동을 꺼도 전기가 계속 들어오는 곳)에 연결한 차량용입니다. "
      "주행 후 시동을 끄고 고른 시간이 지나면 판다가 기기로 가는 전기를 끊어 기기가 꺼지고, 차 배터리를 아낍니다. "
      "다음에 시동을 걸면 판다가 전기를 다시 보내 기기가 켜지도록 합니다. "
      "시동을 끄면 전기도 끊기는 곳에 연결했다면 꺼지지 않음으로 두세요. 꺼지지 않음(처음 값): 기기가 스스로 꺼지지 않습니다. "
      "기기를 켠 뒤 한 번도 시동을 걸지 않았으면 (집에서 업데이트·설정할 때) 꺼지지 않습니다.");
    const QStringList off_names = {"꺼지지 않음", "30초", "1분", "3분", "5분", "10분"};
    const QList<int> off_secs = {0, 30, 60, 180, 300, 600};
    auto off_name = [=]() {
      const int i = off_secs.indexOf(std::atoi(Params().get("OptechOffShutdown").c_str()));
      return "시동 끈 뒤 꺼짐: " + off_names[i < 0 ? 0 : i];
    };
    off_btn->setTitle(off_name());
    QObject::connect(off_btn, &ButtonControl::clicked, [=]() {
      const QString sel = MultiOptionDialog::getSelection("시동 끈 뒤 꺼짐", off_names, off_name().section(": ", 1), this);
      if (sel.isEmpty()) return;
      Params().put("OptechOffShutdown", std::to_string(off_secs[off_names.indexOf(sel)]));
      off_btn->setTitle(off_name());
    });
    advancedToggles->addItem(off_btn);
  }
  advancedToggles->addItem(new ParamControl("OpkrHotspotOnBoot", "부팅 시 핫스팟 켜기", "켜 두면 기기가 켜질 때 핫스팟(테더링)이 자동으로 켜집니다.", SHELL_ICON, this));
  advancedToggles->addItem(carrot_section("주행 기록"));
  auto drivelog_btn = new ButtonControl("주행 기록 보기", "보기", "주행할 때마다 자동으로 남는 기록입니다. 고른 주행에서 켜짐·꺼짐, 경고, 오류, 프로그램 멈춤을 시간순으로 보여 줍니다. (최근 30번 보관)");
  QObject::connect(drivelog_btn, &ButtonControl::clicked, [=]() {
    const QString dir_path = "/data/drivelog";
    QDir dir(dir_path);
    QStringList files = dir.entryList(QStringList() << "2*.txt", QDir::Files, QDir::Name | QDir::Reversed);
    if (files.isEmpty()) {
      ConfirmationDialog::alert("아직 주행 기록이 없어요.", this);
      return;
    }
    // file name: YYYYmmdd-HHMMSS.txt -> "11/18 08:30 주행"
    QStringList labels;
    for (const QString &f : files) {
      labels << QString("%1/%2 %3:%4 주행").arg(f.mid(4, 2), f.mid(6, 2), f.mid(9, 2), f.mid(11, 2));
    }
    const QString sel = MultiOptionDialog::getSelection("볼 주행을 고르세요", labels, "", this);
    if (sel.isEmpty()) return;
    const int idx = labels.indexOf(sel);
    const QString file = files[idx];

    QString first_time, last_time;
    QStringList events;
    const QStringList lines = QString::fromStdString(util::read_file((dir_path + "/" + file).toStdString())).split('\n');
    for (const QString &line : lines) {
      if (line.isEmpty() || line.startsWith("#")) continue;
      const QString t = line.left(5);  // HH:MM
      if (first_time.isEmpty()) first_time = t;
      last_time = t;
      if (line.contains(" ** ")) events << line.left(8) + "  " + line.mid(line.indexOf(" ** ") + 4);
    }

    // crash tracebacks written during this drive (between this file and the next newer one)
    const QString start = file.left(15);
    const QString end = idx > 0 ? files[idx - 1].left(15) : "99999999-999999";
    QStringList crashes;
    for (const QString &c : dir.entryList(QStringList() << "crash-*.txt", QDir::Files, QDir::Name)) {
      const QString when = c.mid(6, 15);  // crash-YYYYmmdd-HHMMSS-<name>.txt
      if (when >= start && when < end) {
        const QStringList tb = QString::fromStdString(util::read_file((dir_path + "/" + c).toStdString())).trimmed().split('\n');
        const QString name = c.mid(22, c.length() - 22 - 4);
        crashes << c.mid(15, 2) + ":" + c.mid(17, 2) + ":" + c.mid(19, 2) + "  " + name + " : " + tb.last();
      }
    }

    QString text = sel + (first_time.isEmpty() ? "" : " (" + first_time + " ~ " + last_time + ")") + "\n\n";
    if (events.isEmpty() && crashes.isEmpty()) {
      text += "경고나 오류 없이 정상이었어요.";
    } else {
      if (!crashes.isEmpty()) text += "[프로그램 오류]\n" + crashes.join("\n") + "\n\n";
      if (events.size() > 200) {
        text += QString("(앞부분 %1줄 생략)\n").arg(events.size() - 200);
        events = events.mid(events.size() - 200);
      }
      text += events.join("\n");
    }
    ConfirmationDialog d(text, "확인", "", true, this);
    d.exec();
  });
  advancedToggles->addItem(drivelog_btn);
  // optech: the drive records to the driver's phone (QR code -> page on this device -> share to the developer)
  auto drivelog_get_btn = new ButtonControl("주행 기록 내려받기", "QR 보기",
    "주행 기록을 폰으로 받아 카카오톡 등으로 보낼 수 있습니다. 누르면 QR 코드가 뜨고, 폰으로 찍으면 기록 목록이 열립니다. "
    "기록에는 켜짐·꺼짐, 경고, 오류와 함께 순정 SCC 상태, 계기판 SCC 안내 신호, 차선 인식, 브레이크등, "
    "openpilot이 보낸 크루즈 버튼(RES 등)이 1초마다 들어 있고, 위치(좌표)는 들어 있지 않습니다. "
    "폰이 이 기기와 같은 와이파이나 핫스팟에 연결돼 있어야 합니다.");
  QObject::connect(drivelog_get_btn, &ButtonControl::clicked, [=]() {
    const QStringList urls = log_page_urls();
    if (urls.isEmpty()) {
      ConfirmationDialog::alert("와이파이나 핫스팟에 연결돼 있지 않아요. 폰 핫스팟에 이 기기를 연결하거나, '부팅 시 핫스팟 켜기'로 "
                                "이 기기의 핫스팟을 켠 뒤 폰을 연결하고 다시 눌러 주세요.", this);
      return;
    }
    Params().put("OptechLogServer", "1");  // logserver.py serves the page for 15 minutes
    LogQrDialog(this, urls).exec();
  });
  advancedToggles->addItem(drivelog_get_btn);
  advancedToggles->addItem(carrot_section("실험 모드"));
  {
    auto op_long = new ParamControl("ExperimentalLongitudinalEnabled", "openpilot 가감속 제어 (알파)", "CAN FD 현대·기아용 롱컨 스위치입니다. 일반 CAN 현대·기아는 이 스위치 대신 토글의 '롱컨'으로 롱컨을 켜지만, 실험 모드 스위치는 이 값이 켜져 있어야 동작합니다. 켜면 자동 긴급 제동(AEB)이 꺼질 수 있습니다.", SHELL_ICON, this);
    op_long->setConfirmation(true, false);
    advancedToggles->addItem(op_long);
    auto e2e = new ParamControl("ExperimentalMode", "실험 모드", "가감속을 항상 E2E(주행 모델이 판단하는 가감속)로 고정합니다. 앞차가 없어도 빨간불·정지선 앞에서 서고 커브 앞에서 줄이지만, 앞차를 따라갈 때도 모델이 가감속을 정해서 레이더 방식(ACC)보다 덜 부드러울 수 있습니다. 상황에 따라 바꾸려면 아래 '자동 E2E / ACC 전환'을 쓰세요(켜 두면 그쪽이 먼저 적용됩니다). 롱컨(openpilot 가감속 제어)일 때만 적용됩니다. 알파 단계 기능이라 실수가 있을 수 있습니다.", SHELL_ICON, this);
    e2e->setConfirmation(true, true);
    advancedToggles->addItem(e2e);
    // OptechAutoE2E (longitudinal_planner.update_auto_e2e, spst dp_long_de2e / dragonpilot dynamic e2e)
    auto auto_e2e = new ParamControl("OptechAutoE2E", "자동 E2E / ACC 전환",
      "상황에 따라 E2E(주행 모델이 판단하는 가감속)와 ACC(레이더로 앞차를 따라가는 가감속)를 자동으로 바꿉니다(optech-spst와 같은 기능).\n"
      "앞차를 따라갈 때는 ACC로 부드럽게 따라가고, 앞차가 없을 때 주행 모델이 '앞에서 서야 한다'고 보면(빨간불·정지선·급한 커브), "
      "정차 중일 때, 시속 75km 아래에서 방향지시등을 켰을 때, 앞차가 갑자기 가까워져 세게 서야 할 때는 E2E로 바꿉니다.\n"
      "켜 두면 '실험 모드'보다 먼저 적용됩니다. 롱컨(openpilot 가감속 제어)일 때만 적용됩니다. 시험 기능이라 항상 주의하세요.",
      SHELL_ICON, this);
    auto_e2e->setConfirmation(true, false);
    advancedToggles->addItem(auto_e2e);
  }
  advancedToggles->addItem(carrot_section("개발용"));
  advancedToggles->addItem(new ParamControl("ShowDebugUI", "디버그 정보", "켜 두면 주행 화면에 개발용 상세 정보를 글자로 보여줍니다.", SHELL_ICON, this));
  // ShowPlotMode (paint.cc ui_draw_plot): 1 accel, 2 lateral accel, 3 speed, 4 model distance / speed, 5 lead speed / accel
  advancedToggles->addItem(choice("ShowPlotMode", "디버그 그래프", "개발용 그래프를 주행 화면에 보여줍니다. 문제를 찾을 때만 씁니다.\n"
    "가속도: 지금 가속도와 목표 가속도\n"
    "커브 횡가속도: 커브에서 옆으로 받는 힘 (지금 값과 계획 값)\n"
    "속도: 지금 속도와 계획 속도\n"
    "주행 모델 거리·속도: 주행 모델이 본 앞쪽 거리와 속도\n"
    "앞차 속도·가속도: 레이더가 잡은 앞차의 속도와 가속도",
    {"안 보임", "가속도", "커브 횡가속도", "속도", "주행 모델 거리·속도", "앞차 속도·가속도"}));

  toggles_layout->addWidget(startToggles);
  toggles_layout->addWidget(cruiseToggles);
  toggles_layout->addWidget(speedToggles);
  toggles_layout->addWidget(steerToggles);
  toggles_layout->addWidget(dispToggles);
  toggles_layout->addWidget(advancedToggles);
  ScrollView* toggles_view = new ScrollView(toggles, this);
  carrotLayout->addWidget(toggles_view, 1);

  homeScreen->setLayout(carrotLayout);
  main_layout->addWidget(homeScreen);
  main_layout->addWidget(selectCar);
  main_layout->setCurrentWidget(homeScreen);

  togglesCarrot(0);

}


void CarrotPanel::togglesCarrot(int widgetIndex) {
  startToggles->setVisible(widgetIndex == 0);
  cruiseToggles->setVisible(widgetIndex == 1);
  speedToggles->setVisible(widgetIndex == 2);
  steerToggles->setVisible(widgetIndex == 3);
  dispToggles->setVisible(widgetIndex == 4);
  advancedToggles->setVisible(widgetIndex == 5);
  refreshDmIcon();
}

void CarrotPanel::refreshDmIcon() {
  const bool basic = optech_dm_mode() == 0;
  dmIcon->setEnabled(basic);
  dmIcon->setToggled(basic && std::atoi(Params().get("ShowDmInfo").c_str()) >= 1);
}

void CarrotPanel::showEvent(QShowEvent *event) {
  QWidget::showEvent(event);
  refreshDmIcon();
  for (auto &refresh : choice_refresh) refresh();
}

// optech (user 2026-11-26, same as optech-spst c7adbf7): every item with several named values is '이름: 지금 값' + 선택
// (list dialog) instead of a number with -/+; the param still holds the number (index into names)
ButtonControl *CarrotPanel::choice(const QString &param, const QString &title, const QString &desc, const QStringList &names,
                                   int def, std::function<void(int)> changed) {
  ButtonControl *btn = new ButtonControl("", "선택", desc);
  const std::string key = param.toStdString();
  auto current = [=]() {
    const std::string v = Params().get(key);
    const int i = std::atoi(v.c_str());
    return (v.empty() || i < 0 || i >= names.size()) ? def : i;
  };
  auto refresh = [=]() { btn->setTitle(title + ": " + names[current()]); };
  QObject::connect(btn, &ButtonControl::clicked, [=]() {
    const QString sel = MultiOptionDialog::getSelection(title, names, names[current()], this);
    if (sel.isEmpty()) return;
    const int i = names.indexOf(sel);
    Params().put(key, std::to_string(i));
    refresh();
    if (changed) changed(i);
  });
  refresh();
  choice_refresh.append(refresh);
  return btn;
}

void CarrotPanel::updateButtonStyles() {
  QString styleSheet = R"(
      #tab_btn_0, #tab_btn_1, #tab_btn_2, #tab_btn_3, #tab_btn_4, #tab_btn_5 {
        height: 120px; border-radius: 15px; background-color: #393939;
      }
      #tab_btn_0:pressed, #tab_btn_1:pressed, #tab_btn_2:pressed, #tab_btn_3:pressed, #tab_btn_4:pressed, #tab_btn_5:pressed {
        background-color: #4a4a4a;
      }
  )";
  styleSheet += QString("#tab_btn_%1 { background-color: #33ab4c; }").arg(currentCarrotIndex);
  setStyleSheet(styleSheet);
}

// ajouatom
CValueControl::CValueControl(const QString& params, const QString& title, const QString& desc, const QString& icon, int min, int max, int unit/*=1*/) : AbstractControl(title, desc, icon)
{

    m_params = params;
    m_min = min;
    m_max = max;
    m_unit = unit;

    label.setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    label.setStyleSheet("color: #e0e879");
    hlayout->addWidget(&label);

    btnminus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
    btnplus.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");
    btnminus.setFixedSize(110, 100);
    btnplus.setFixedSize(110, 100);
    hlayout->addWidget(&btnminus);
    hlayout->addWidget(&btnplus);

    QObject::connect(&btnminus, &QPushButton::released, [=]() {
        auto str = QString::fromStdString(Params().get(m_params.toStdString()));
        int value = str.toInt();
        value = value - m_unit;
        if (value < m_min) {
            value = m_min;
        }
        else {
        }

        //UIScene& scene = uiState()->scene;//QUIState::ui_state.scene;
        //scene.scr.autoFocus = value;
        QString values = QString::number(value);
        Params().put(m_params.toStdString(), values.toStdString());
        refresh();
    });

    QObject::connect(&btnplus, &QPushButton::released, [=]() {
        auto str = QString::fromStdString(Params().get(m_params.toStdString()));
        int value = str.toInt();
        value = value + m_unit;
        if (value > m_max) {
            value = m_max;
        }
        else {
        }

        //UIScene& scene = uiState()->scene;//QUIState::ui_state.scene;
        //scene.scr.autoFocus = value;
        QString values = QString::number(value);
        Params().put(m_params.toStdString(), values.toStdString());
        refresh();
    });
    refresh();
}
TimeZoneSelectCombo::TimeZoneSelectCombo() : AbstractControl("TZ", "", "../assets/offroad/icon_shell.png") 
{
  combobox.setStyleSheet(R"(
    subcontrol-origin: padding;
    subcontrol-position: top left;
    selection-background-color: #111;
    selection-color: yellow;
    color: white;
    background-color: #393939;
    border-style: solid;
    border: 0px solid #1e1e1e;
    border-radius: 0;
    width: 100px;
  )");

  combobox.addItem(tr("시간대를 선택하세요"));
  QFile timezonelistfile("/data/openpilot/selfdrive/assets/addon/param/TimeZone");
  if (timezonelistfile.open(QIODevice::ReadOnly)) {
    QTextStream timezonename(&timezonelistfile);
    while (!timezonename.atEnd()) {
      QString line = timezonename.readLine();
      combobox.addItem(line);
    }
    timezonelistfile.close();
  }

  combobox.setFixedWidth(950);

  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn.setFixedSize(150, 100);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    if (btn.text() == tr("해제")) {
      if (ConfirmationDialog::confirm(tr("기본값으로 되돌릴까요?"), tr("Reboot"), this)) {
        params.put("OPKRTimeZone", "UTC");
        combobox.setCurrentIndex(0);
        refresh();
      }
    }
  });

  //combobox.view()->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  hlayout->addWidget(&combobox, Qt::AlignLeft);
  hlayout->addWidget(&btn, Qt::AlignRight);

  QObject::connect(&combobox, static_cast<void(QComboBox::*)(int)>(&QComboBox::activated), [=](int index)
  {
    combobox.itemData(combobox.currentIndex());
    QString str = combobox.currentText();
    if (combobox.currentIndex() != 0) {
      if (ConfirmationDialog::confirm(tr("확인을 누르면 시간대를 다음으로 설정합니다:") + "\n" + str, tr("Reboot"), this)) {
        params.put("OPKRTimeZone", str.toStdString());
      }
    }
    refresh();
  });
  refresh();
}

void TimeZoneSelectCombo::refresh() {
  QString selected_timezonename = QString::fromStdString(params.get("OPKRTimeZone"));
  int index = combobox.findText(selected_timezonename);
  if (index >= 0) combobox.setCurrentIndex(index);
  if (selected_timezonename.length()) {
    btn.setEnabled(true);
    btn.setText(tr("해제"));
  } else {
    btn.setEnabled(false);
    btn.setText(tr("설정"));
  }
}

void CValueControl::refresh()
{
    QString value = QString::fromStdString(Params().get(m_params.toStdString()));
    label.setText(value);
    emit valueChanged(value.toInt());
    btnminus.setText("－");
    btnplus.setText("＋");
}

GitHash::GitHash() : AbstractControl("커밋(로컬/리모트)", "", "") {

    QString lhash = QString::fromStdString(Params().get("GitCommit").substr(0, 10));
    QString rhash = QString::fromStdString(Params().get("GitCommitRemote").substr(0, 10));
    hlayout->addStretch(2);

    local_hash.setText(QString::fromStdString(Params().get("GitCommit").substr(0, 10)));
    remote_hash.setText(QString::fromStdString(Params().get("GitCommitRemote").substr(0, 10)));
    local_hash.setAlignment(Qt::AlignVCenter);
    remote_hash.setAlignment(Qt::AlignVCenter);
    local_hash.setStyleSheet("color: #aaaaaa");
    if (lhash == rhash) {
        remote_hash.setStyleSheet("color: #aaaaaa");
    }
    else {
        remote_hash.setStyleSheet("color: #0099ff");
    }
    hlayout->addWidget(&local_hash);
    hlayout->addWidget(&remote_hash);
}
SelectCar::SelectCar(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  // Back button: from a brand's cars back to the brands, from the brands back to the settings
  QPushButton* back = new QPushButton(tr("Back"));
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  // menu file lines "CAR|brand|name" in brand, model and year order (car_menu.py). The brand list comes first; a brand
  // shows only its cars. Each car row keeps its CAR string in UserRole; that is what SelectedCar stores.
  QStringList lines = get_list_utf8("/data/params/d/SupportedCars");
  lines += get_list_utf8("/data/params/d/SupportedCars_gm");
  auto brands = std::make_shared<QStringList>();
  auto cars = std::make_shared<QMap<QString, QList<QPair<QString, QString>>>>();  // brand -> (CAR, name)
  for (const QString &line : lines) {
    const QStringList parts = line.split('|');
    const QString car = parts[0];
    if (car.isEmpty()) continue;
    const QString brand = parts.size() >= 3 ? parts[1] : QString("OTHER");
    const QString name = parts.size() >= 3 ? parts[2] : car;
    if (!brands->contains(brand)) brands->append(brand);
    (*cars)[brand].append(qMakePair(car, name));
  }
  static const QMap<QString, QString> BRAND_KO = {{"HYUNDAI", "현대"}, {"KIA", "기아"}, {"GENESIS", "제네시스"}, {"CHEVROLET", "쉐보레"},
                                                  {"GMC", ""}, {"CADILLAC", "캐딜락"}, {"BUICK", "뷰익"},
                                                  {"TOYOTA", "토요타"}, {"LEXUS", "렉서스"}, {"HONDA", "혼다"}, {"ACURA", "어큐라"},
                                                  {"VOLKSWAGEN", "폭스바겐"}, {"AUDI", "아우디"}, {"SEAT", "세아트"}, {"SKODA", "스코다"}, {"MAZDA", "마쓰다"},
                                                  {"NISSAN", "닛산"}, {"SUBARU", "스바루"},
                                                  {"CHRYSLER", "크라이슬러"}, {"JEEP", "지프"}};
  auto cur_brand = std::make_shared<QString>();

  auto show_brands = [=]() {
    cur_brand->clear();
    list->clear();
    const QString selected = QString::fromStdString(Params().get("SelectedCar"));
    QListWidgetItem* none_item = new QListWidgetItem(tr("[ 선택 안 함 ]"));
    none_item->setData(Qt::UserRole, QString());
    list->addItem(none_item);
    list->setCurrentItem(none_item);
    for (const QString &b : *brands) {
      const QString ko = BRAND_KO.value(b);
      QListWidgetItem* item = new QListWidgetItem("   " + b + (ko.isEmpty() ? "" : "  " + ko) +
                                                  QString("   (%1)   ▶").arg((*cars)[b].size()));
      item->setData(Qt::UserRole + 1, b);
      list->addItem(item);
      for (const auto &c : (*cars)[b]) {
        if (c.first == selected) list->setCurrentItem(item);
      }
    }
    list->scrollToTop();
  };

  auto show_cars = [=](const QString &b) {
    *cur_brand = b;
    list->clear();
    const QString selected = QString::fromStdString(Params().get("SelectedCar"));
    const QString ko = BRAND_KO.value(b);
    QListWidgetItem* header = new QListWidgetItem("■ " + b + (ko.isEmpty() ? "" : "  " + ko));
    header->setFlags(Qt::NoItemFlags);  // not selectable
    header->setForeground(QColor("#33ab4c"));
    QFont f = header->font();
    f.setBold(true);
    header->setFont(f);
    list->addItem(header);
    for (const auto &c : (*cars)[b]) {
      QListWidgetItem* item = new QListWidgetItem("      " + c.second);
      item->setData(Qt::UserRole, c.first);
      list->addItem(item);
      if (c.first == selected) list->setCurrentItem(item);
    }
    if (list->currentItem()) list->scrollToItem(list->currentItem(), QAbstractItemView::PositionAtCenter);
    else list->scrollToTop();
  };

  show_brands();

  connect(back, &QPushButton::clicked, [=]() {
    if (!cur_brand->isEmpty()) {
      show_brands();
    } else {
      emit backPress();
    }
  });

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){
    const QString brand = item->data(Qt::UserRole + 1).toString();
    if (!brand.isEmpty()) {  // a brand: show its cars
      show_cars(brand);
      return;
    }
    if (!(item->flags() & Qt::ItemIsSelectable)) return;  // brand header
    const QString car = item->data(Qt::UserRole).toString();
    if (car.isEmpty())
        Params().remove("SelectedCar");
    else
        Params().put("SelectedCar", car.toStdString());

    show_brands();  // next time the list starts from the brands again
    emit selectedCar();
    });

  main_layout->addWidget(list);
}
// ---- 레이 EV 설정 / 포터 II EV 설정 pages ----
// CAR names must match NON_SCC_CAR in selfdrive/car/hyundai/values.py and the param prefixes PEDAL_PARAMS in
// selfdrive/car/hyundai/pedal.py; tests/test_nonscc_port_c2.py checks them.
struct CcOnlyCarEntry {
  const char *car, *title, *prefix;
  int pedal_max_default, pedal_lo, pedal_hi;
  bool speed_buttons;  // openpilot can press the cruise buttons (Hyundai/Kia only)
  const char *intro, *pedal_desc, *note;
};
static const char *NON_SCC_INTRO =
  "스마트 크루즈(SCC)가 없는 차량입니다. 페달 장치가 없으면 openpilot은 조향만 하고, 속도는 차의 크루즈 컨트롤이 유지합니다. "
  "앞차를 감지하는 레이더가 없으니 앞차와의 거리는 항상 운전자가 지켜 주세요. "
  "아직 실제 차로 시험하지 않았으니, 처음에는 'openpilot 사용'을 끄고(읽기 전용) 짧게 달려 본 뒤 조향을 켜 주세요.";
static const char *NON_SCC_PEDAL =
  "comma 페달 장치를 단 경우에만 켜세요. 켜면 openpilot이 가속페달로 가감속을 맡고, 차의 크루즈는 자동으로 꺼집니다. "
  "브레이크 장치가 없어 감속은 회생제동만큼만 되고, 앞차 뒤에서 멈추거나 정차를 유지할 수 없습니다. "
  "페달 장치가 인식되지 않으면 켜 두어도 조향만 합니다. 재부팅해야 적용됩니다.";
static const CcOnlyCarEntry CC_ONLY_CARS[] = {
  {"KIA RAY EV 2024", "레이 EV", "Ray", 70, 10, 70, true, NON_SCC_INTRO, NON_SCC_PEDAL,
   "페달 보정값은 스타파일럿이 실제 레이 EV로 맞춘 값을 씁니다(당근파일럿도 같은 값). 아래 '페달 세게'와 '속도 오차 보정'은 당근파일럿 방식입니다."},
  {"HYUNDAI PORTER II EV 2024", "포터 II EV", "Porter", 40, 10, 70, true, NON_SCC_INTRO, NON_SCC_PEDAL,
   "포터는 아직 실제 차로 맞춘 페달 보정값이 없어 레이 EV 값을 임시로 씁니다. 안전한 곳에서 낮은 상한으로 먼저 확인하고, 필요하면 보정값을 맞춰야 합니다."},
  // (the Chevrolet Bolt EV page was removed 2026-11-23: optech-hkg is Hyundai / Kia / Genesis only)
};

int CcOnlyCarPage::count() { return sizeof(CC_ONLY_CARS) / sizeof(CC_ONLY_CARS[0]); }
QString CcOnlyCarPage::carName(int index) { return CC_ONLY_CARS[index].car; }
QString CcOnlyCarPage::menuTitle(int index) { return QString(CC_ONLY_CARS[index].title) + " 설정"; }

CcOnlyCarPage::CcOnlyCarPage(int index, QWidget* parent): QWidget(parent) {
  const CcOnlyCarEntry &e = CC_ONLY_CARS[index];
  const QString prefix = e.prefix;

  // shown inside the 시작 tab (its scroll view), so no back button or scroll view of its own
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  ListWidget* list = new ListWidget(this);
  list->addItem(carrot_section(QString(e.title) + " 설정"));
  QLabel* intro = new QLabel(e.intro);
  intro->setWordWrap(true);
  intro->setStyleSheet("font-size: 38px; color: #E0E0E0; padding: 10px;");
  list->addItem(intro);
  if (e.speed_buttons) {
    list->addItem(new ParamControl(prefix + "SpeedButtons", "크루즈 속도 자동 조절",
      "페달을 쓰지 않을 때, openpilot이 크루즈 버튼을 대신 눌러 설정 속도를 바꿉니다(커브·과속카메라 감속 등). 크루즈가 브레이크를 세게 걸지 못하므로 앞차와의 거리는 항상 운전자가 지켜야 합니다. 기본은 꺼짐(조향만).", SHELL_ICON, this));
  }

  list->addItem(carrot_section("페달 (comma 페달 장치)"));
  list->addItem(new ParamControl(prefix + "Pedal", "페달 사용", e.pedal_desc, SHELL_ICON, this));
  list->addItem(new CValueControl(prefix + "PedalMax", "페달 최대 밟는 양",
    QString("openpilot이 가속페달을 최대 몇 %까지 밟을지 정합니다. 처음에는 낮게 시작해 보세요. %1~%2, 기본 %3. ")
      .arg(e.pedal_lo).arg(e.pedal_hi).arg(e.pedal_max_default) + e.note,
    ROAD_ICON, e.pedal_lo, e.pedal_hi, 5));
  if (prefix == "Ray") {
    list->addItem(new ParamControl("RayPedalStrong", "페달 세게 (당근)",
      "같은 명령에 페달을 약 2배 더 밟습니다. 당근파일럿에서 2024년식 레이 EV 차주가 '가속이 약하다'고 해서 바꾼 값입니다. "
      "당근파일럿의 주행 기록 분석에서는 이 설정으로 감속해야 할 때 오히려 가속한 경우가 있었으니, 가속이 정말 약할 때만 켜고 처음에는 조심해서 확인하세요. "
      "기본은 꺼짐(스타파일럿 값). 재부팅해야 적용됩니다.", SHELL_ICON, this));
    list->addItem(new CValueControl("RayPedalKi", "속도 오차 보정 (Ki)",
      "설정 속도와 실제 속도가 계속 어긋날 때(오르막에서 처지거나 내리막에서 빨라질 때) 조금씩 페달을 더하거나 빼서 맞춥니다(당근파일럿 방식). "
      "값 ×0.001이 Ki입니다. 0이면 끔, 기본 50(당근파일럿 레이 추천값). 너무 크면 속도가 출렁입니다. 재부팅해야 적용됩니다.",
      ROAD_ICON, 0, 200, 10));
  }

  main_layout->addWidget(list);
}
