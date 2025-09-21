/**
 * @file sonar_monitor.cpp
 * @brief Implementation of the SonarMonitor module.
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "modules/sensors/sonar.h"

#include <cstdint>
#include <cstdlib>  // Ensure stdlib.h is available
#include <cstring>  // Ensure proper inclusion for strncpy

#include "TimerOne.h"
#include "common/core/serial_manager.h"

namespace WimbleRobotics_Teensy {

SonarMonitor* SonarMonitor::instance_ = nullptr;

SonarMonitor& SonarMonitor::getInstance() {
  if (!instance_) {
    instance_ = new SonarMonitor();
  }
  return *instance_;
}

SonarMonitor::SonarMonitor() : WimbleRobotics_Teensy::Module() {
  // Initialize sensor configurations, All that is needed is to disable them by default.
  for (uint8_t i = 0; i < kMaxSonarSensors; ++i) {
    sensor_configs_[i].enabled = false;
  }
}

void SonarMonitor::configureSensor(uint8_t index, const SonarSensorConfig& config) {
  if (index < kMaxSonarSensors) {
    sensor_configs_[index] = config;
    strncpy(sensor_status_[index].name, config.name.c_str(),
            sizeof(sensor_status_[index].name) - 1);
    sensor_status_[index].name[sizeof(sensor_status_[index].name) - 1] =
        '\0';  // Ensure null-termination
  }
}

void SonarMonitor::setup() {
  for (uint8_t i = 0; i < kMaxSonarSensors; ++i) {
    if (sensor_configs_[i].enabled) {
      // Set up the GPIO pins.
      pinMode(sensor_configs_[i].trigger_pin, OUTPUT);
      pinMode(sensor_configs_[i].echo_pin, INPUT);
      
      // Attach a single interrupt handler for all echo pins
      attachInterrupt(digitalPinToInterrupt(sensor_configs_[i].echo_pin), echo_interrupt_handler,
                      CHANGE);
    }
  }

  // Set up Timer1 which will use a state machine to trigger sensors in round-robin fashion.
  Timer1.initialize(timer_interval_us);
  Timer1.attachInterrupt(timerInterruptHandler);
}

void SonarMonitor::timerInterruptHandler() {
  static int32_t counter = echo_sample_interval_us;
  SonarMonitor& instance = SonarMonitor::getInstance();

  // Serial.print("State: ");
  // Serial.print((uint8_t)instance.state_);
  // Serial.print(", counter: ");
  // Serial.println(counter);
  
  switch (instance.state_) {
    case State::PULSE_HIGH: {
      // Find the next enabled sensor
      for (int i = 0; i < kMaxSonarSensors; ++i) {
        instance.current_sensor_index_ = (instance.current_sensor_index_ + 1) % kMaxSonarSensors;
        if (instance.sensor_configs_[instance.current_sensor_index_].enabled) {
          // Serial.print("Switching to sensor ");
          // Serial.println(instance.current_sensor_index_);
          break;
        }
      }

      if (!instance.sensor_configs_[instance.current_sensor_index_].enabled) {
        // No sensors enabled, nothing to do
        Serial.println("No sensors enabled, skipping trigger.");
        return;
      }

      // Trigger the next sensor in a round-robin fashion
      // Serial.print("Triggering sensor ");
      // Serial.println(instance.current_sensor_index_);
      digitalWrite(instance.sensor_configs_[instance.current_sensor_index_].trigger_pin, HIGH);
      instance.state_ = State::PULSE_LOW;
    }

    break;

    case State::PULSE_LOW: {
      // Serial.println("PULSE_LOW");
      digitalWrite(instance.sensor_configs_[instance.current_sensor_index_].trigger_pin, LOW);
      counter = initial_echo_sample_interval_count;
      instance.state_ = State::COUNTDOWN;
    }

    break;

    case State::COUNTDOWN: {
      // Wait for echo handlin g via interrupt
      // Serial.print("Counter: ");
      // Serial.println(counter);
      if (--counter > 0)
        break;

      instance.state_ = State::PULSE_HIGH;
      // Serial.println("Echo timeout, moving to next sensor.");
    }
  }
}

void SonarMonitor::loop() {
  // The only thing for loop to do is print out distances periodically.
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 200) {
    for (int i = 0; i < kMaxSonarSensors; ++i) {
      const SonarSensorStatus& status = SonarMonitor::getInstance().getSensorStatus(i);
      if (status.is_valid) {
        String msg =
            "Sonar " + String(status.name) + " distance: " + String(status.distance_cm) + " cm";
        SerialManager::getInstance().sendMessage("SONAR", msg.c_str());
      }
    }

    last_print_time = millis();
  }
}

void SonarMonitor::handleEcho() {
  uint8_t voltage_level = digitalRead(sensor_configs_[current_sensor_index_].echo_pin);
  if (voltage_level == HIGH) {
    echo_start_time_ = micros();
    is_echo_active_ = true;
  } else {
    if (is_echo_active_) {
      unsigned long echo_duration = micros() - echo_start_time_;
      sensor_status_[current_sensor_index_].distance_cm = echo_duration * 0.034 / 2;
      sensor_status_[current_sensor_index_].is_valid = true;
      sensor_status_[current_sensor_index_].last_reading_time_ms = millis();
      is_echo_active_ = false;
      Serial.print("echo duration (us): ");
      Serial.print(echo_duration);
      Serial.print(", distance (cm): ");
      Serial.println(sensor_status_[current_sensor_index_].distance_cm);
    } else {
      Serial.print("Echo LOW on sensor ");
      Serial.println(current_sensor_index_);
      Serial.println("But no active echo - spurious interrupt?");
    }

    SonarMonitor::getInstance().current_sensor_index_ =
        (SonarMonitor::getInstance().current_sensor_index_ + 1) % kMaxSonarSensors;
  }
}

void SonarMonitor::echo_interrupt_handler() {
  if (instance_) {
    instance_->handleEcho();
  }
}

float SonarMonitor::getDistance(uint8_t sensor_index) const {
  if (sensor_index < kMaxSonarSensors) {
    return sensor_status_[sensor_index].distance_cm;
  }
  return -1.0f;
}

const SonarSensorStatus& SonarMonitor::getSensorStatus(uint8_t sensor_index) const {
  static SonarSensorStatus invalid_status;
  if (sensor_index < kMaxSonarSensors) {
    return sensor_status_[sensor_index];
  }
  return invalid_status;
}

}  // namespace WimbleRobotics_Teensy
