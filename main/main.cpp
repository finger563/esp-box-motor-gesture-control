#include <chrono>
#include <thread>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_pm.h"

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

extern "C" esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data,
                                            size_t size, wifi_pkt_rx_ctrl_t *rx_ctrl) {
  // ESP_PARAM_CHECK(src_addr);
  // ESP_PARAM_CHECK(data);
  // ESP_PARAM_CHECK(size);
  // ESP_PARAM_CHECK(rx_ctrl);

  // static uint32_t count = 0;

  // ESP_LOGI(TAG, "espnow_recv, <%" PRIu32 "> [" MACSTR "][%d][%d][%u]: %.*s",
  //          count++, MAC2STR(src_addr), rx_ctrl->channel, rx_ctrl->rssi, size, size, (char *)data);

  return ESP_OK;
}

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

  esp_event_loop_create_default();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());

  espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
  espnow_init(&espnow_config);

  espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, on_esp_now_recv);

  // make a task to read out the IMU data and print it to console
  using namespace std::chrono_literals;
  espp::Timer imu_timer({.period = 10ms,
                         .callback = [&gravity, &gravity_mutex]() -> bool {
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
                           static constexpr float beta =
                               0.1f; // higher = more accelerometer, lower = more gyro
                           static espp::MadgwickFilter f(beta);

                           // update the state
                           f.update(dt, accel.x, accel.y, accel.z, gyro.x * M_PI / 180.0f,
                                    gyro.y * M_PI / 180.0f, gyro.z * M_PI / 180.0f);
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
                         },
                         .auto_start = true,
                         .task_config = {
                             .name = "IMU",
                             .stack_size_bytes = 6 * 1024,
                             .priority = 10,
                             .core_id = 0,
                         }});

  // make a task to send the command (based on gravity vector) to the motor using esp-now
  espp::Timer motor_timer({.period = 10ms,
                           .callback = [&gravity, &gravity_mutex]() -> bool {
                             std::array<float, 3> g_vector;
                             {
                               std::lock_guard<std::mutex> lock(gravity_mutex);
                                g_vector = gravity;
                             }

                             espnow_frame_head_t frame_head;
                             memset(&frame_head, 0, sizeof(espnow_frame_head_t));
                             frame_head.retransmit_count = CONFIG_RETRY_NUM;
                             frame_head.broadcast = true;

                             static uint8_t data[10] = {0};
                             size_t size = 10;
                             espnow_send(ESPNOW_DATA_TYPE_DATA, ESPNOW_ADDR_BROADCAST, data, size, &frame_head, portMAX_DELAY);

                             return false;
                           },
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
