# Using the HC‑SR04 Ultrasonic Sensor with Teensy 4.1 for ROS 2 (Jazzy)

This tutorial shows how to drive HC‑SR04 “ping” sensors from an Arduino‑compatible MCU (Teensy 4.1), and how to architect the firmware for reliable, high‑rate distance reporting suitable for ROS 2 navigation stacks.

Audience: ROS 2 developers with modest C++ skills, comfortable flashing microcontrollers and reading wiring diagrams.

---

## What the HC‑SR04 does (quick refresher)

- You provide a 10 µs HIGH pulse on TRIG.
- The module emits 8 ultrasonic wavefronts.
- ≈475 µs after TRIG goes LOW, ECHO goes HIGH to open the “listening window.****”****
- When a reflection is detected, ECHO goes LOW; the HIGH duration is the round‑trip time.
- Distance (meters) = (echo_duration_µs × 343 m/s) ÷ 2 = echo_duration_µs × 0.000343 ÷ 2.
- If no echo arrives: the cheap module I tested holds ECHO HIGH until about 134 ms, even though the acoustic max‑range (~4 m) implies ≈38 ms. Plan for ~134 ms worst‑case.

> Note: Speed of sound varies (~331 m/s + 0.6 m/s·°C). Use 343 m/s at 20 °C unless you compensate.

---

## Hardware: wiring for one or more sensors

- Power: 5 V and GND from Teensy power rail. Ensure common ground with Teensy. Keep in mind that a single HC-SR04 can draw about 15 milliamps of current so make sure your 5 V source can supply all of the sensors. As usual, avoid ground loops when wiring all of your devices.
- Logic:
  - TRIG → Teensy GPIO (output).
  - ECHO → Teensy GPIO (input). Many HC‑SR04 boards output 5 V ECHO; use a resistor divider or level shifter for 3.3 V MCUs like Teensy 4.1. I'm using my custom, Wimble Robotics Teensy Sensor board which has level shifted ports for up to 4 SONAR sensors.
- Multiple sensors: give each its own TRIG/ECHO pins. Avoid triggering multiple sensors simultaneously to reduce cross‑talk.

Example (Teensy 4.1):
- Sensor 0: TRIG=32, ECHO=33
- Sensor 1: TRIG=30, ECHO=31

---

## Timing model you’ll implement

Two time‑sensitive activities happen:

1) Trigger-signal shaping and round‑robin scheduling
- A periodic timer interrupt runs at a fixed interval (this repo uses 10 µs). It drives a tiny state machine that:
  - raises TRIG for 10 µs
  - lowers TRIG, then waits up to the valid echo window (~40 ms)
  - if ECHO reported a hit, immediately advances to the next sensor
  - if no echo, waits out the module’s hardware timeout (~134 ms), then advances
- Trade‑off: 10 µs period = precise pulse shaping and fine timing but higher CPU load. A slower tick (50–100 µs, even 1 ms with micros() deadlines) is a valid alternative if you reduce work in the ISR. Note that the spec says that the trigger signal must be held high for a minimum of 10 10 µs with no maximum time specified. The Teensy 4.1 with its high clock rate can easily handle this. Slower Arduinos may need to use a lower rate clock so the CPU isn't overloaded.

  Note that there isn't any real requirement to do all of this in a timer handler. You could easily raise the trigger signal in a normal loop, do a 10 µs delay and lower the trigger signal. The code in this example is trying to achieve fast frame rates for the SONAR sensors. The Nav2 stack relies on regular, high-speed sensor frames for best performance and this is an example of one way to achieve that.

2) Echo edge capture (interrupt per ECHO pin)
- On ECHO rising edge: store start time (micros()).
- On ECHO falling edge: compute duration, convert to distance, flag result as ready.
- Best practice: attach a distinct ISR per ECHO pin so you know exactly which sensor reported. A single shared ISR can work if you map pin→device inside the ISR. That's what is demoed in this code--one ISR for all sensors--just so you can see how it can be done.

---

## Distance math

- Constant speed of sound: `kSpeed_m_per_us = 0.000343f` (343 m/s). Note that you could adjust the speed of sound by pairing it with a temperature sensor, but the use of sensors for obstacle detection in the Nav2 stack is unlikely to benefit from knowing that, e.g., an obstacle is 13.353 millimeters away instead of 13.4 millimeters.
- Distance (meters): `distance = (duration_us * kSpeed_m_per_us) / 2.0f`. Remember that the time reported by the sensor is round trip time, the obstacle is half that distance away.
- Example: 1283 µs → 0.1283 ms → ≈0.439 m → 43.9 cm.

---

## Firmware architecture (what this repo does)

I'm not publishing the complete code repository here. I will publish a more complete example at some time in the future, but my code for the Sigyn robot has to deal with a more complete set of engineering considerations and that requires a much longer discussion of how to you deal with sensors, safety, fault handling, recovery, logging, performance, repeatability, accuracy and similar issues for a non-toy robot. This code example relies on a pair of other code modules, ***Module*** and ***SerialManager** which are used without much discussion here. 

***Module*** provides a plug-in service where you can merely instantiate a piece of code to register a new class of service, such as time-of-flight sensors, sonar sensors, motor drivers, etc. It also provides a wrapper service for performance measurement and helps guarantee required performance for critical systems.

***SerialManager*** provides a single-point or responsibility for communicating between the Teensy microcontroller and the ROS 2 system.

The main pieces of the architecture in this example are:

- Module: `SonarMonitor` manages up to 4 sensors in round‑robin.
- Timer: periodic interrupt (10 µs) drives a small state machine to issue TRIG pulses and wait for echo windows.
- ISR: a handler records echo timing and computes distance.
- Reporting: the main loop periodically prints distances over a shared `SerialManager` (ROS 2 bridge on the host can parse and publish to topics).

Key constants you can tune (see `modules/sensors/sonar.h`):
- `timer_interval_us = 10`
- `echo_sample_interval_us = 40'000`  // 40 ms valid window
- `max_sample_interval_us = 150'000`   // 150 ms to cover observed 134 ms no‑echo case

---

## Minimal usage pattern (pseudo‑C++)

```cpp
// Configure one sensor
Device dev;
dev.name = "FrontSONAR";
dev.trigger_pin = 32;
dev.echo_pin = 33;
dev.enabled = true;
SonarMonitor::getInstance().configureSensor(0, dev);

// In setup()
Module::setupAll();

// In loop()
Module::loopAll();
// When a new reading is available, send it via SerialManager (see repo code)
```

Tips:
- Configure all sensors before calling `Module::setupAll()` so pins/interrupts get attached.
- One ISR per ECHO pin is preferred for robust mapping.

---

## Performance considerations on Teensy 4.1

- 10 µs ISR tick means 100 k ISR/s, so keep ISR extremely small (toggle a pin, decrement counters, set flags). Move logging and heavy math to the main loop.
- Consider replacing tick counters with micros()-based deadlines; you can then use a slower timer or no timer ISR at all and do scheduling from loop().
- Avoid heap churn: prefer fixed‑size `char` buffers over `String` for names and logging on embedded devices. In fact, in my code base, I regularly analyze the code and eliminate all heap-based uses.

---

## ROS 2 integration sketch

On the host PC (ROS 2 Jazzy):
- Run a small bridge node that reads serial lines like `SONAR:name=FrontSONAR,d=0.73` and publishes `sensor_msgs/Range` per sensor.
- Recommended topic layout: `/sonar/front`, `/sonar/left`, … with appropriate frame_ids.
- Rate‑limit publishes to only when a new measurement arrives (edge‑triggered), not at a fixed interval.

Message example from firmware:
```
SONAR:name=FrontSONAR,meters=0.73
```

---

## Troubleshooting

- ECHO always high (~134 ms): no echo within window; verify wiring, level shifting, and that only one sensor is active at a time.
- Spurious readings: ensure sensors aren’t firing simultaneously; increase time between triggers or use acoustic isolation.

---

## Next steps and enhancements

- Switch from a shared ISR to one ISR per ECHO pin.
- Replace 10 µs tick with a lighter scheduler using micros() deadlines.
- Add temperature compensation for speed of sound.
- Convert `Device.name` and log pathways to fixed buffers to avoid `String` fragmentation.
- Emit ROS‑friendly JSON or CSV lines consistently from firmware.

---

## Appendix: Key equations and limits

- Max valid echo window (~4 m): ≈38 ms
- Observed ECHO timeout on some HC‑SR04 boards: ≈134 ms
- Speed of sound (20 °C): 343 m/s ≈ 0.0343 cm/µs
- Distance (cm): `cm = duration_us * 0.0343 / 2`

```text
TRIG:  ┌──────────┐
       │   10µs   │
       └──────────┘
ECHO:           ┌───────────────────────┐
                │  listen window (H)   │  High until echo or timeout
                └───────┬──────────────┘
                        └→ falling edge → measure duration
```
