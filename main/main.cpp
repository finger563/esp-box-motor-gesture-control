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

enum class EspNowCtrlStatus { INIT, BOUND, MAX };
static EspNowCtrlStatus espnow_ctrl_status = EspNowCtrlStatus::INIT;

static void init_wifi();
static void espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id,
                                 void *event_data);
static esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data, size_t size,
                                 wifi_pkt_rx_ctrl_t *rx_ctrl);
static void app_responder_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                                       espnow_attribute_t responder_attribute, uint32_t status);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Motor Gesture Control", .level = espp::Logger::Verbosity::DEBUG});

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
        // on long press, start binding for esp-now
        if (espnow_ctrl_status == EspNowCtrlStatus::INIT) {
          espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, true);
          espnow_ctrl_status = EspNowCtrlStatus::BOUND;
        }
      }

      if (last_double_press_state) {
        logger.info("Double press detected");
        // on double press, reset the esp-now binding
        espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, false);
        espnow_ctrl_status = EspNowCtrlStatus::INIT;

        // reset the double press state
        last_double_press_state = false;
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

  // gravity vector for motor position control with IMU
  std::mutex gravity_mutex;
  std::array<float, 3> gravity = {0, 0, 0};

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
  auto imu_timer_cb = [&gravity, &gravity_mutex]() -> bool {
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
  auto motor_timer_cb = [&gravity, &gravity_mutex]() -> bool {
    std::array<float, 3> g_vector;
    {
      std::lock_guard<std::mutex> lock(gravity_mutex);
      g_vector = gravity;
    }

    if (espnow_ctrl_status == EspNowCtrlStatus::BOUND) {
      espnow_frame_head_t frame_head;
      memset(&frame_head, 0, sizeof(espnow_frame_head_t));
      frame_head.retransmit_count = CONFIG_RETRY_NUM;
      frame_head.broadcast = true;

      static uint8_t data[10] = {0};
      size_t size = 10;
      espnow_send(ESPNOW_DATA_TYPE_DATA, ESPNOW_ADDR_BROADCAST, data, size, &frame_head,
                  portMAX_DELAY);
    }

    return false;
  };
  espp::Timer motor_timer({.period = 10ms,
                           .callback = motor_timer_cb,
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
    [[maybe_unused]] espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
    // ESP_LOGI(TAG, "bind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac),
    // info->initiator_attribute);
    // TODO: we are now bound, indicate it and start the sending
    break;
  }

  case ESP_EVENT_ESPNOW_CTRL_BIND_ERROR: {
    [[maybe_unused]] espnow_ctrl_bind_error_t *bind_error = (espnow_ctrl_bind_error_t *)event_data;
    // ESP_LOGW(TAG, "bind error: %s", bind_error_to_string(*bind_error));
    break;
  }

  case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
    [[maybe_unused]] espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
    // ESP_LOGI(TAG, "unbind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac),
    // info->initiator_attribute); we are now unbound, indicate it and stop the sending
    break;
  }

  default:
    break;
  }
}

esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data, size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl) {
  // ESP_PARAM_CHECK(src_addr);
  // ESP_PARAM_CHECK(data);
  // ESP_PARAM_CHECK(size);
  // ESP_PARAM_CHECK(rx_ctrl);

  // static uint32_t count = 0;

  // ESP_LOGI(TAG, "espnow_recv, <%" PRIu32 "> [" MACSTR "][%d][%d][%u]: %.*s",
  //          count++, MAC2STR(src_addr), rx_ctrl->channel, rx_ctrl->rssi, size, size, (char
  //          *)data);

  return ESP_OK;
}

void app_responder_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                                espnow_attribute_t responder_attribute, uint32_t status) {
  // TODO: handle the control data
}
