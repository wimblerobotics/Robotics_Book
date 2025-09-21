/**
 * @file sonar_monitor.cpp
 * @brief Implementation of the SonarMonitor module.
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "modules/sensors/sonar.h"
#include <cstring> // Ensure proper inclusion for strncpy
#include <cstdlib> // Ensure stdlib.h is available

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
  // Initialize sensor configurations
  for (uint8_t i = 0; i < kMaxSonarSensors; ++i) {
    sensor_configs_[i].enabled = false;
  }
}

void SonarMonitor::configureSensor(uint8_t index, const SonarSensorConfig& config) {
  Serial.print("Configuring sensor ");
  Serial.println(index);
  Serial.print("Trigger pin: ");
  Serial.println(config.trigger_pin);
  Serial.print("Echo pin: ");
  Serial.println(config.echo_pin);
  Serial.print("Enabled: ");
  Serial.println(config.enabled);
  if (index < kMaxSonarSensors) {
    sensor_configs_[index] = config;
    // Use strncpy with c_str() to safely copy the name
    strncpy(sensor_status_[index].name, config.name.c_str(), sizeof(sensor_status_[index].name) - 1);
    sensor_status_[index].name[sizeof(sensor_status_[index].name) - 1] = '\0'; // Ensure null-termination
  }
}

void SonarMonitor::setup() {
  for (uint8_t i = 0; i < kMaxSonarSensors; ++i) {
    if (sensor_configs_[i].enabled) {
      pinMode(sensor_configs_[i].trigger_pin, OUTPUT);
      pinMode(sensor_configs_[i].echo_pin, INPUT);
      // Attach a single interrupt handler for all echo pins
      attachInterrupt(digitalPinToInterrupt(sensor_configs_[i].echo_pin), echo_interrupt_handler,
                      CHANGE);
    }
  }
}

void SonarMonitor::loop() {
  if (!sensor_configs_[current_sensor_index_].enabled)
    for (int i = 0; i < kMaxSonarSensors; ++i) {
      current_sensor_index_ = (current_sensor_index_ + 1) % kMaxSonarSensors;
      if (sensor_configs_[current_sensor_index_].enabled) {
        break;
      }
    }

  if (!sensor_configs_[current_sensor_index_].enabled) {
    // No sensors enabled, nothing to do
    return;
  }

  // Trigger the next sensor in a round-robin fashion
  if (sensor_configs_[current_sensor_index_].enabled) {
    triggerSensor(current_sensor_index_);
  }

  // A delay is needed to allow the sensor to process the ping
  delay(150);
}

void SonarMonitor::triggerSensor(uint8_t index) {
  digitalWrite(sensor_configs_[index].trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor_configs_[index].trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor_configs_[index].trigger_pin, LOW);
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
