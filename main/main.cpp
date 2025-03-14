#include <chrono>
#include <thread>

#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "esp_mac.h"

#include "espnow.h"
#include "espnow_ctrl.h"
#include "espnow_storage.h"
#include "espnow_utils.h"

#include "esp-box.hpp"
#include "foc_utils.hpp"
#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"
#include "timer.hpp"
#include "vector2d.hpp"

#include "gui.hpp"

#include "command.hpp"

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "BOX", .level = espp::Logger::Verbosity::INFO});

/////////////////////////////
// IMU Data and Functions
/////////////////////////////

// gravity vector for motor position control with IMU
static std::mutex gravity_mutex;
static std::array<float, 3> gravity = {0, 0, 0};
static std::atomic<float> current_angle = 0.0f;
static std::atomic<bool> reset_angle = false;

// current control mode for the motor
static std::atomic<CommandCode> control_mode = CommandCode::STOP;

// how many RPM we want to add for each full rotation
static constexpr float RPM_PER_ROTATION = 200.0f;
static float angle_to_speed(float angle);

/////////////////////////////
// ESP-NOW Data and Functions
/////////////////////////////

enum class EspNowCtrlStatus { INIT, BOUND };
static EspNowCtrlStatus espnow_ctrl_status = EspNowCtrlStatus::INIT;

static void init_wifi();
static void espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id,
                                 void *event_data);
static esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data, size_t size,
                                 wifi_pkt_rx_ctrl_t *rx_ctrl);
static void app_responder_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                                       espnow_attribute_t responder_attribute, uint32_t status);
static constexpr const char *bind_error_to_string(espnow_ctrl_bind_error_t bind_error);

extern "C" void app_main(void) {
  logger.info("Bootup");

  espp::EspBox &box = espp::EspBox::get();
  box.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Running on {}", box.box_type());

  // initialize the sound
  if (!box.initialize_sound()) {
    logger.error("Failed to initialize sound!");
    return;
  }
  // initialize the LCD
  if (!box.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = box.lcd_width() * 50;
  // initialize the LVGL display for the esp-box
  if (!box.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  std::shared_ptr<Gui> gui;
  auto gui_config = Gui::Config{.on_bond_button_pressed =
                                    [&]() {
                                      logger.info("Starting esp-now binding");
                                      espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, true);
                                      espnow_ctrl_status = EspNowCtrlStatus::BOUND;
                                      gui->show_control_screen();
                                    },
                                .on_enabled_checkbox_checked =
                                    [&](bool checked) {
                                      logger.info("Motors enabled: {}", checked);
                                      if (checked) {
                                        // get the dropdown to set the control mode
                                        auto index = gui->get_control_drop_down_value();
                                        control_mode = (CommandCode)(index + 2);
                                      } else {
                                        // set the control mode to off
                                        control_mode = CommandCode::STOP;
                                      }
                                      logger.info("Control mode: {}", control_mode.load());
                                    },
                                .on_control_dropdown_value_changed =
                                    [&](int index) {
                                      logger.info("Control mode index: {}", index);
                                      // Only update the control mode if the enabled checkbox
                                      // is set
                                      if (gui->get_enabled_check_box_checked()) {
                                        // SET_ANGLE = 2, SET_SPEED = 3
                                        control_mode = (CommandCode)(index + 2);
                                      }
                                      logger.info("Control mode: {}", control_mode.load());
                                    },
                                .log_level = espp::Logger::Verbosity::INFO};
  gui = std::make_shared<Gui>(gui_config);

  // initialize the touchpad
  if (!box.initialize_touch(nullptr)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // initialize the IMU
  if (!box.initialize_imu()) {
    logger.error("Failed to initialize IMU!");
    return;
  }

  // unmute the audio and set the volume to 60%
  box.mute(false);
  box.volume(60.0f);

  // set the display brightness to be 75%
  box.brightness(75.0f);

  // initialize the wifi and esp-now stacks
  espnow_storage_init();

  init_wifi();

  espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
  espnow_init(&espnow_config);
  espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, on_esp_now_recv);
  esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, espnow_event_handler, NULL);

  ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
  espnow_ctrl_responder_data(app_responder_ctrl_data_cb);

  // make a task to read out the IMU data and print it to console
  auto imu_timer_cb = [&gui]() -> bool {
    static auto &box = espp::EspBox::get();
    static auto imu = box.imu();

    auto now = esp_timer_get_time(); // time in microseconds
    static auto t0 = now;
    auto t1 = now;
    float dt = (t1 - t0) / 1'000'000.0f; // convert us to s
    t0 = t1;

    std::error_code ec;
    // get imu data
    auto accel = imu->get_accelerometer(ec);
    auto gyro = imu->get_gyroscope(ec);
    // auto temp = imu->get_temperature(ec);

    // with only the accelerometer + gyroscope, we can't get yaw :(
    float roll = 0, pitch = 0, yaw = 0; // NOTE:yaw is unused
    static constexpr float beta = 0.1f; // higher = more accelerometer, lower = more gyro
    static espp::MadgwickFilter f(beta);

    // update the state
    f.update(dt, accel.x, accel.y, accel.z, gyro.x * M_PI / 180.0f, gyro.y * M_PI / 180.0f,
             gyro.z * M_PI / 180.0f);
    f.get_euler(roll, pitch, yaw);
    pitch *= M_PI / 180.0f;
    roll *= M_PI / 180.0f;

    // compute the gravity vector and store it
    float gx = sin(pitch);
    float gy = -cos(pitch) * sin(roll);
    float gz = -cos(pitch) * cos(roll);

    auto box_type = box.box_type();
    if (box_type == espp::EspBox::BoxType::BOX) {
      float tmp = -gx;
      gx = gy;
      gy = tmp;
    }

    // compute the angle of the device with respect to the y axis
    espp::Vector2f grav_2d(gx, gy);
    // only update the angle if the magnitude of the 2d gravity vector is above
    // 0.3
    if (grav_2d.magnitude() > 0.3f) {
      espp::Vector2f y_axis(0.0f, 1.0f);
      // this produces a value between [-pi, pi]
      float angle = y_axis.signed_angle(grav_2d);
      // this is used to know if we've crossed the -pi/pi boundary
      static float prev_angle = angle;
      // this is used to know how many times we've crossed the -pi/pi boundary
      // so that we can properly update the current angle
      static float angle_offset = 0.0f;

      // we need to track if we crossed the -pi/pi boundary, so that we can
      // properly update our actual angle, which should be continuous
      if (angle - prev_angle > M_PI) {
        angle_offset -= 2 * M_PI;
      } else if (angle - prev_angle < -M_PI) {
        angle_offset += 2 * M_PI;
      }
      // update the previous angle
      prev_angle = angle;
      // compute the current angle
      current_angle = angle + angle_offset;
    }

    logger.debug("Current Angle: {:.2f} deg", current_angle.load() * 180.0f / M_PI);

    std::lock_guard<std::mutex> lock(gravity_mutex);
    gravity[0] = gx;
    gravity[1] = gy;
    gravity[2] = gz;

    return false;
  };
  using namespace std::chrono_literals;
  espp::Timer imu_timer({.period = 10ms,
                         .callback = imu_timer_cb,
                         .auto_start = true,
                         .task_config = {
                             .name = "IMU",
                             .stack_size_bytes = 6 * 1024,
                             .priority = 20,
                             .core_id = 1,
                         }});

  // make a task to send the command (based on gravity vector) to the motor using esp-now
  auto command_timer_cb = []() -> bool {
    if (espnow_ctrl_status == EspNowCtrlStatus::BOUND) {
      espnow_frame_head_t frame_head;
      memset(&frame_head, 0, sizeof(espnow_frame_head_t));
      frame_head.retransmit_count = CONFIG_RETRY_NUM;
      frame_head.broadcast = true;

      // use the control mode to determine whether we send direct angle set
      // point or convert the angle into a velocity
      auto angle = current_angle.load();

      Command command;

      switch (control_mode) {
      case CommandCode::STOP:
        command.code = CommandCode::STOP;
        break;
      case CommandCode::SET_ANGLE:
        // send the angle as the set point (radians)
        command.code = CommandCode::SET_ANGLE;
        command.angle_radians = angle;
        break;
      case CommandCode::SET_SPEED:
        // Convert the angle (radians) into a velocity. we want one full
        // rotation to equate to 60 RPM,which is 1 rotation per second, or 2pi
        // rad/s. The controller expects speed targets in RAD/S. We can use
        // espp::RPM_TO_RADS to help with this conversion.
        command.code = CommandCode::SET_SPEED;
        command.speed_radians_per_second = angle_to_speed(angle);
        break;
      default:
        // we don't know what this is, so set the motor to off
        command.code = CommandCode::STOP;
        break;
      }

      // ensure we only send the data if the command is different enough from
      // the last sent command
      static Command prev_command = {};
      if (command == prev_command) {
        return false;
      }
      // store the command for comparison
      prev_command = command;

      // now send it
      auto size = command.size();
      espnow_send(ESPNOW_DATA_TYPE_DATA, ESPNOW_ADDR_BROADCAST,
                  reinterpret_cast<uint8_t *>(&command), size, &frame_head, portMAX_DELAY);
    }

    return false;
  };
  espp::Timer command_timer({.period = 10ms,
                             .callback = command_timer_cb,
                             .auto_start = true,
                             .task_config = {
                                 .name = "Motor",
                                 .stack_size_bytes = 6 * 1024,
                                 .priority = 10,
                                 .core_id = 1,
                             }});

  while (true) {
    // low priority: update the gui image rotation using the current angle
    auto image_angle = current_angle.load();
    gui->set_image_rotation(image_angle);
    // now set the target label text
    bool control_is_enabled = gui->get_enabled_check_box_checked();
    std::string label_text = "Target: N/A";
    if (control_is_enabled) {
      bool control_is_angle = control_mode == CommandCode::SET_ANGLE;
      std::string units = control_is_angle ? "rad" : "rad/sec";
      float target = control_is_angle ? image_angle : angle_to_speed(image_angle);
      label_text = fmt::format("Target: {:.1f} {}", target, units);
    }
    gui->set_target_label_text(label_text);
    std::this_thread::sleep_for(30ms);
  }
}

void init_wifi() {
  esp_event_loop_create_default();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
  if (base != ESP_EVENT_ESPNOW) {
    return;
  }

  switch (id) {
  case ESP_EVENT_ESPNOW_CTRL_BIND: {
    espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
    logger.info("Bound to {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, initiator_type: {}",
                MAC2STR(info->mac), (int)info->initiator_attribute);
    break;
  }

  case ESP_EVENT_ESPNOW_CTRL_BIND_ERROR: {
    espnow_ctrl_bind_error_t *bind_error = (espnow_ctrl_bind_error_t *)event_data;
    logger.warn("Bind error: {}", bind_error_to_string(*bind_error));
    break;
  }

  case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
    espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
    logger.info("Unbound from {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", MAC2STR(info->mac));
    break;
  }

  default:
    break;
  }
}

esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl) {
  logger.info("Received data from {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", MAC2STR(src_addr));
  logger.info("RSSI: {}", (int)rx_ctrl->rssi);
  uint8_t *data_ptr = reinterpret_cast<uint8_t *>(data);
  logger.info("Data: {::02x}", std::vector<uint8_t>(data_ptr, data_ptr + size));

  return ESP_OK;
}

void app_responder_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                                espnow_attribute_t responder_attribute, uint32_t status) {
  // TODO: handle the control data
}

constexpr const char *bind_error_to_string(espnow_ctrl_bind_error_t bind_error) {
  switch (bind_error) {
  case ESPNOW_BIND_ERROR_NONE:
    return "No error";
  case ESPNOW_BIND_ERROR_TIMEOUT:
    return "bind timeout";
  case ESPNOW_BIND_ERROR_RSSI:
    return "bind packet RSSI below expected threshold";
  case ESPNOW_BIND_ERROR_LIST_FULL:
    return "bindlist is full";
  default:
    return "unknown error";
  }
}

static float angle_to_speed(float angle) {
  // Convert the angle (radians) into a velocity. we want one full
  // rotation to equate to 60 RPM,which is 1 rotation per second, or 2pi
  // rad/s. The controller expects speed targets in RAD/S. We can use
  // espp::RPM_TO_RADS to help with this conversion.
  return (angle / espp::_2PI) * RPM_PER_ROTATION * espp::RPM_TO_RADS;
}
