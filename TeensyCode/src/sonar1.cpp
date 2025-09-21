/**
 * @file WimbleRobotics_Teensysonar.cpp
 * @brief Example usage of the SonarMonitor module.
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include <Arduino.h>

#include "common/core/serial_manager.h"
#include "modules/sensors/sonar.h"

using namespace WimbleRobotics_Teensy;

void setup() {
  // Initialize SerialManager
  SerialManager::getInstance().initialize(115200);
  SerialManager::getInstance().sendDiagnosticMessage("INFO", "main", "Serial manager initialized.");

  // Configure SonarMonitor
  SonarMonitor& sonar_monitor = SonarMonitor::getInstance();
  SonarSensorConfig sonar_config;
  sonar_config.enabled = true;
  sonar_config.trigger_pin = 32;  // Example pin
  sonar_config.echo_pin = 33;     // Example pin
  sonar_config.name = "FrontSONAR"; // Safely set the name

  sonar_monitor.configureSensor(0, sonar_config);

  // Setup all registered modules
  Module::setupAll();

  SerialManager::getInstance().sendDiagnosticMessage("INFO", "main", "Setup complete.");
}

void loop() {
  // Loop all registered modules
  Module::loopAll();

  // Example of getting and printing sonar data
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 200) {
    for (int i = 0; i < kMaxSonarSensors; ++i) {
      const SonarSensorStatus& status = SonarMonitor::getInstance().getSensorStatus(i);
      if (status.is_valid) {
        String msg = "Sonar " + String(status.name) + " distance: " + String(status.distance_cm) + " cm";
        SerialManager::getInstance().sendMessage("SONAR", msg.c_str());
      }
    }

    last_print_time = millis();
  }
}