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
#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"
#include "timer.hpp"
#include "vector2d.hpp"

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "BOX", .level = espp::Logger::Verbosity::DEBUG});

/////////////////////////////
// IMU Data and Functions
/////////////////////////////

// gravity vector for motor position control with IMU
static std::mutex gravity_mutex;
static std::array<float, 3> gravity = {0, 0, 0};
static float current_angle = 0.0f;
static std::atomic<bool> reset_angle = false;

// current control mode for the motor
enum class ControlMode { OFF, POSITION, VELOCITY };
static ControlMode control_mode = ControlMode::OFF;

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
  espp::Task::BaseConfig display_task_config = {
      .name = "Display",
      .stack_size_bytes = 6 * 1024,
      .priority = 10,
      .core_id = 0,
  };
  // initialize the LVGL display for the esp-box
  if (!box.initialize_display(pixel_buffer_size, display_task_config)) {
    logger.error("Failed to initialize display!");
    return;
  }

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

  // define a callback function for when the boot button is pressed
  auto button_callback = [&](const auto &event) {
    // track the button state change time to determine single press or double
    // press
    static uint64_t last_press_time_us = 0;
    static bool last_double_press_state = false;

    bool pressed = event.active;

    // if the button is pressed
    if (pressed) {
      // get the current time
      auto current_time_us = esp_timer_get_time();
      auto time_since_last_press_us = current_time_us - last_press_time_us;
      // if the button was pressed within 500ms of the last press, it is a
      // double press
      if (time_since_last_press_us < 500'000) {
        last_double_press_state = true;
      } else {
        last_double_press_state = false;
      }
      // update the last press time
      last_press_time_us = current_time_us;
    } else {
      // if the button was released
      auto hold_time_us = esp_timer_get_time() - last_press_time_us;
      // if the button was pressed for more than 500ms, it is a long press
      if (hold_time_us > 500'000) {
        logger.info("Long press detected");
        // on long press, reset the esp-now binding
        if (espnow_ctrl_status == EspNowCtrlStatus::BOUND) {
          logger.info("Resetting esp-now binding");
          espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, false);
          espnow_ctrl_status = EspNowCtrlStatus::INIT;
        }
      } else if (last_double_press_state) {
        logger.info("Double press detected");
        // on double press, start binding for esp-now
        if (espnow_ctrl_status == EspNowCtrlStatus::INIT) {
          logger.info("Starting esp-now binding");
          espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, true);
          espnow_ctrl_status = EspNowCtrlStatus::BOUND;
        }
        // reset the double press state
        last_double_press_state = false;
      } else {
        logger.info("Single press detected");
      }
    }
  };
  // initialize the esp-box boot button for input, passing the callback
  if (!box.initialize_boot_button(button_callback)) {
    logger.error("Failed to initialize boot button!");
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
  auto imu_timer_cb = []() -> bool {
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

    logger.info("Current Angle: {:.2f} deg", current_angle * 180.0f / M_PI);

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
                             .priority = 10,
                             .core_id = 0,
                         }});

  // make a task to send the command (based on gravity vector) to the motor using esp-now
  auto command_timer_cb = []() -> bool {
    espp::Vector2f grav_2d{};
    {
      std::lock_guard<std::mutex> lock(gravity_mutex);
      grav_2d.x(gravity[0]);
      grav_2d.y(gravity[1]);
    }

    if (espnow_ctrl_status == EspNowCtrlStatus::BOUND) {
      espnow_frame_head_t frame_head;
      memset(&frame_head, 0, sizeof(espnow_frame_head_t));
      frame_head.retransmit_count = CONFIG_RETRY_NUM;
      frame_head.broadcast = true;

      static uint8_t data[3] = {0};
      size_t size = sizeof(data);
      // TODO: update
      // data[0] = (uint8_t)(g_vector[0] * 127.0f + 127.0f);
      // data[1] = (uint8_t)(g_vector[1] * 127.0f + 127.0f);
      // data[2] = (uint8_t)(g_vector[2] * 127.0f + 127.0f);
      // static uint8_t last_sent_data[3] = {0};
      // if (memcmp(data, last_sent_data, size) == 0) {
      //   // no change in data, don't send
      //   return false;
      // }
      // memcpy(last_sent_data, data, size);
      espnow_send(ESPNOW_DATA_TYPE_DATA, ESPNOW_ADDR_BROADCAST, data, size, &frame_head,
                  portMAX_DELAY);
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
                                 .core_id = 0,
                             }});

  while (true) {
    std::this_thread::sleep_for(1s);
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
