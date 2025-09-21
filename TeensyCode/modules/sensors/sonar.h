/**
 * @file sonar_monitor.h
 * @brief Sonar system for the Wimble Robotics Teensy Ecosystem
 *
 * Provides sensor readings for HC-SR04 ultrasonic distance sensors, following
 * the modular architecture defined in `common/core/module.h`. This module
 * supports multiple sensors, real-time distance monitoring, and integration
 * with the system's safety and diagnostic reporting features.
 *
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <Arduino.h>

#include <cstdint>

#include "common/core/module.h"
#include "common/core/serial_manager.h"

namespace WimbleRobotics_Teensy {

// Maximum number of sonar sensors supported
static constexpr uint8_t kMaxSonarSensors = 4;

/**
 * @brief Configuration for a single sonar sensor.
 */
struct Device {
  String name;
  uint8_t trigger_pin;
  uint8_t echo_pin;
  bool enabled = false;
  int32_t countdown_ticks = 0;  // For timing the echo.
  bool echo_found = false;
  int32_t waitout_countdown_ticks = 0;
  float distance = 0.0f;

  Device(String name, uint8_t trigger_pin, uint8_t echo_pin)
      : name(name), trigger_pin(trigger_pin), echo_pin(echo_pin), enabled(true) {}

  Device() = default;
};

/**
 * @brief Sonar monitoring module for the Teensy system.
 *
 * This module manages multiple HC-SR04 sonar sensors, providing distance
 * readings and status information. It follows the Teensy module architecture
 * for easy integration and real-time performance.
 */
class SonarMonitor : public WimbleRobotics_Teensy::Module {
 public:
  static SonarMonitor& getInstance();

  // Sensor access
  float getDistance(uint8_t sensor_index) const;

  // Configuration
  void configureSensor(uint8_t index, const Device& config);

 protected:
  // Module interface
  void setup() override;
  void loop() override;
  const char* name() const override { return "SonarMonitor"; }

 private:
  static constexpr float max_distance_meters = 2.2f;  // Maximum distance for HC-SR04 is 2.2 meters

  // timer_intervale_us must be the desired pulse width (10us) for HC-SR04.
  static constexpr uint32_t timer_interval_us = 10;  // 10 us timer interval

  // max_sample_interval_us is the longest sample interval for a sonar sensor.
  // When the HC-SR04 doesn't get an echo, the devices I tested ended
  // up taking a full 134 milliseconds before the echo signal dropped.
  // I rounded this up to 150 milliseconds.
  // This is the period you need to wait before triggering the sensor again.
  static constexpr uint32_t max_sample_interval_us = 150'000;  // 150 ms between samples

  // echo_sample_interval_us is the longest valid sample interval for a sonar sensor.
  // The HC-SR04 has a valid maximum round trip time of 38 milliseconds.
  // I rounded this up to 40 milliseconds.
  // If the echo hasn't been received in this time, we assume no echo.
  static constexpr uint32_t echo_sample_interval_us = 40'000;  // 40 ms between samples

  enum class State { PULSE_HIGH, PULSE_LOW, COUNTDOWN, WAIT_OUT_ECHO } state_ = State::PULSE_HIGH;

  SonarMonitor();

  static void timerInterruptHandler();

  void handleEcho();

  Device devices_[kMaxSonarSensors];

  volatile uint8_t current_sensor_index_ = 0;
  volatile unsigned long echo_start_time_ = 0;
  volatile bool is_echo_active_ = false;

  static void echo_interrupt_handler();
  static SonarMonitor* instance_;
};

}  // namespace WimbleRobotics_Teensy
