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
struct SonarSensorConfig {
  String name;
  uint8_t trigger_pin;
  uint8_t echo_pin;
  bool enabled = true;
};

/**
 * @brief Status of a single sonar sensor.
 */
struct SonarSensorStatus {
  float distance_cm = -1.0f;
  bool is_valid = false;
  char name[16] = "Unnamed";
  uint32_t last_reading_time_ms = 0;
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
  const SonarSensorStatus& getSensorStatus(uint8_t sensor_index) const;

  // Configuration
  void configureSensor(uint8_t index, const SonarSensorConfig& config);

protected:
  // Module interface
  void setup() override;
  void loop() override;
  const char* name() const override { return "SonarMonitor"; }

private:
  SonarMonitor();

  void triggerSensor(uint8_t index);
  void handleEcho();

  SonarSensorConfig sensor_configs_[kMaxSonarSensors];
  SonarSensorStatus sensor_status_[kMaxSonarSensors];
  
  volatile uint8_t current_sensor_index_ = 0;
  volatile unsigned long echo_start_time_ = 0;
  volatile bool is_echo_active_ = false;

  static void echo_interrupt_handler();
  static SonarMonitor* instance_;
};

} // namespace wimblerobotics_teensy
