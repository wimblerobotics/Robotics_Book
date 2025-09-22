/**
 * @file WimbleRobotics_Teensysonar.cpp
 * @brief Example usage of the SonarMonitor module.
 *
 * @author Wimble Robotics
 * @date 2025
 * @license Apache-2.0
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
  sonar_monitor.configureSensor(0, Device("Front-SONAR", 32, 33));

  // Setup all registered modules
  Module::setupAll();

  SerialManager::getInstance().sendDiagnosticMessage("INFO", "main", "Setup complete.");
}

void loop() {
  // Loop all registered modules
  Module::loopAll();
}