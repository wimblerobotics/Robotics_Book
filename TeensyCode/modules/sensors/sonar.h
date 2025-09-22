/**
 * @file sonar_monitor.h
 * @brief Sonar system for the Wimble Robotics Teensy Ecosystem
 *
 * Provides sensor readings for HC-SR04 ultrasonic distance sensors, following
 * the modular architecture defined in `common/core/module.h`. This module
 * supports multiple sensors, real-time distance monitoring, and integration
 * with the system's safety and diagnostic reporting features.
 *
 * Design highlights
 * - Timing-sensitive work (TRIG pulse shaping and echo windows) is executed in a
 *   timer-driven state machine to minimize jitter regardless of work in `loop()`.
 * - ECHO timing is captured via GPIO interrupts to achieve microsecond precision
 *   with minimal ISR work.
 * - A round-robin scheduler ensures only one sensor is active at a time, reducing
 *   cross-talk and providing a steady per-sensor frame cadence.
 *
 * Notes on HC-SR04 behavior
 * - TRIG must be held HIGH for at least 10 microseconds.
 * - Valid acoustic round-trip window is ~38 ms (about 4 meters). Many boards will
 *   hold ECHO HIGH far longer (observed ~134 ms) if no echo returns. We explicitly
 *   wait out this interval before re-triggering to avoid interfering with the module.
 *
 * This implementation favors clarity and predictable timing over micro-optimizations.
 * It is suitable as a reference and can be extended with per-pin ISRs or hardware
 * input-capture for production systems requiring additional robustness.
 *
 * @author Wimble Robotics
 * @date 2025
 * @license Apache-2.0
 */

#pragma once

#include <Arduino.h>

#include <cstdint>

#include "common/core/module.h"
#include "common/core/serial_manager.h"

namespace WimbleRobotics_Teensy {

// Maximum number of sonar sensors supported. Increase carefully—more sensors
// will increase aggregate cycle time and may require adjusting timing constants.
static constexpr uint8_t kMaxSonarSensors = 4;

/**
 * @brief Configuration for a single sonar sensor.
 */
// Per-device configuration and latest reading/state. `name` is used for reporting.
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
  // Maximum distance used when no echo is detected (clamp for reporting).
  static constexpr float max_distance_meters = 2.2f;  // HC-SR04 usable max distance

  // Timer tick in microseconds. Also used as the exact TRIG width (10 µs).
  static constexpr uint32_t timer_interval_us = 10;  // 10 µs timer interval

  // Longest allowed sample interval for a sonar sensor.
  // When the HC-SR04 doesn't get an echo, observed ECHO HIGH can be ~134 ms.
  // We round up to 150 ms to ensure the device has fully timed out before re-triggering.
  static constexpr uint32_t max_sample_interval_us = 150'000;  // 150 ms between samples

  // Longest valid echo window. If no echo within 40 ms, treat as no-echo for measurement
  // purposes and proceed to wait-out state to complete to 150 ms.
  static constexpr uint32_t echo_sample_interval_us = 40'000;  // 40 ms between samples

  // Timer-driven state machine for precise TRIG pulse generation and echo timing.
  enum class State { PULSE_HIGH, PULSE_LOW, COUNTDOWN, WAIT_OUT_ECHO } state_ = State::PULSE_HIGH;

  SonarMonitor();

  // ISR: advances state machine and schedules sensors in round-robin.
  static void timerInterruptHandler();

  // ISR: handles ECHO rising/falling edges for the current sensor.
  void handleEcho();

  Device devices_[kMaxSonarSensors];

  // Shared across ISRs: index of currently active sensor, timestamp of echo start,
  // and a flag indicating an active echo window.
  volatile uint8_t current_sensor_index_ = 0;
  volatile unsigned long echo_start_time_ = 0;
  volatile bool is_echo_active_ = false;

  // Trampoline for attachInterrupt to invoke instance handler.
  static void echo_interrupt_handler();
  static SonarMonitor* instance_;
};

}  // namespace WimbleRobotics_Teensy
