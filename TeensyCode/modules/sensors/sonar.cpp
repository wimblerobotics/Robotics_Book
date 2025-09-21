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
    devices_[i].enabled = false;
  }
}

void SonarMonitor::configureSensor(uint8_t index, const Device& device) {
  if (index < kMaxSonarSensors) {
    devices_[index] = device;
  }
}

void SonarMonitor::setup() {
  for (uint8_t i = 0; i < kMaxSonarSensors; ++i) {
    if (devices_[i].enabled) {
      // Set up the GPIO pins.
      pinMode(devices_[i].trigger_pin, OUTPUT);
      pinMode(devices_[i].echo_pin, INPUT);

      // Attach a single interrupt handler for all echo pins
      attachInterrupt(digitalPinToInterrupt(devices_[i].echo_pin), echo_interrupt_handler, CHANGE);
    }
  }

  // Set up Timer1 which will use a state machine to trigger sensors in round-robin fashion.
  Timer1.initialize(timer_interval_us);
  Timer1.attachInterrupt(timerInterruptHandler);
}

void SonarMonitor::timerInterruptHandler() {
  SonarMonitor& instance = SonarMonitor::getInstance();

  switch (instance.state_) {
    case State::PULSE_HIGH: {
      // Find the next enabled sensor
      for (int i = 0; i < kMaxSonarSensors; ++i) {
        instance.current_sensor_index_ = (instance.current_sensor_index_ + 1) % kMaxSonarSensors;
        if (instance.devices_[instance.current_sensor_index_].enabled) {
          break;
        }
      }

      if (!instance.devices_[instance.current_sensor_index_].enabled) {
        // No sensors enabled, nothing to do
        return;
      }

      // Trigger the next sensor in a round-robin fashion
      // Serial.print("Triggering sensor ");
      // Serial.println(instance.current_sensor_index_);
      digitalWrite(instance.devices_[instance.current_sensor_index_].trigger_pin, HIGH);
      instance.state_ = State::PULSE_LOW;
    }

    break;

    case State::PULSE_LOW: {
      Device& device = instance.devices_[instance.current_sensor_index_];
      digitalWrite(device.trigger_pin, LOW);
      device.countdown_ticks = echo_sample_interval_us / timer_interval_us;
      device.echo_found = false;
      device.waitout_countdown_ticks =
          (max_sample_interval_us - echo_sample_interval_us) / timer_interval_us;
      instance.state_ = State::COUNTDOWN;
    }

    break;

    case State::COUNTDOWN: {
      // Wait for echo handlin g via interrupt
      Device& device = instance.devices_[instance.current_sensor_index_];

      if (device.echo_found) {
        // Echo received, move to next sensor
        instance.state_ = State::PULSE_HIGH;
        break;
      }

      if (--instance.devices_[instance.current_sensor_index_].countdown_ticks > 0)
        break;

      // No echo received within the valid interval, move to waitout state
      instance.state_ = State::WAIT_OUT_ECHO;
    }

    break;

    case State::WAIT_OUT_ECHO: {
      Device& device = instance.devices_[instance.current_sensor_index_];

      if (device.echo_found) {
        // Echo received, move to next sensor
        instance.state_ = State::PULSE_HIGH;
        break;
      }

      if (--device.waitout_countdown_ticks > 0)
        break;

      // No echo received within the maximum interval, mark as invalid
      device.echo_found = false;
      device.distance = max_distance_meters;

      // Move to the next sensor
      instance.state_ = State::PULSE_HIGH;
    }
  }
}

void SonarMonitor::loop() {
  // The only thing for loop to do is print out distances periodically.
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 200) {
    for (int i = 0; i < kMaxSonarSensors; ++i) {
      Device& device = devices_[i];
      if (device.enabled) {
        String msg =
            "Sonar " + String(device.name) + " distance: " + String(device.distance) + " meters";
        SerialManager::getInstance().sendMessage("SONAR", msg.c_str());
      }
    }

    last_print_time = millis();
  }
}

void SonarMonitor::handleEcho() {
  Device& device = devices_[current_sensor_index_];
  if (!device.enabled) {
    return;
  }

  uint8_t voltage_level = digitalRead(device.echo_pin);
  if (voltage_level == HIGH) {
    echo_start_time_ = micros();
    is_echo_active_ = true;
  } else {
    if (is_echo_active_) {
      device.echo_found = true;
      unsigned long echo_duration = micros() - echo_start_time_;
      device.distance = echo_duration * 0.000343 / 2;
      is_echo_active_ = false;
    } else {
      Serial.print("Echo LOW on sensor ");
      Serial.println(current_sensor_index_);
      Serial.println("But no active echo - spurious interrupt?");
    }
  }
}

void SonarMonitor::echo_interrupt_handler() {
  if (instance_) {
    instance_->handleEcho();
  }
}

float SonarMonitor::getDistance(uint8_t sensor_index) const {
  if (sensor_index < kMaxSonarSensors) {
    const Device& device = devices_[sensor_index];
    return device.distance;
  }
  return max_distance_meters;
}

}  // namespace WimbleRobotics_Teensy
