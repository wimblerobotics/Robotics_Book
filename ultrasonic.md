# How Does the SONAR Device Work

1. Set TRIGGER signal high.
2. Wait 10 microseconds.
3. Set TRIGGER signal low.
4. The ultrasonic will send 8 waves of ultrasonic sound.
5. The ECHO signal will go high.
6. When the echo sound returns, the echo signal will go low. If no echo comes after 38 milliseconds, the ECHO signal will go low then.

Given that sound travels about 340 meters per second, in 38 milliseconds, a sound wave will have traveled 12.92 meters. Since the sound wave must have traveled out and reflected back to be detected, the maximum distance you can detect is half of 12.92 meters, or 6.47 meters.

# Overview of How The Code Will Work

1. Build an interrupt handler for each SONAR device's ECHO signal. The interrupt handler will be called whenever the ECHO signal changes. So handler will be called once when ECHO goes high after the 8 waves of ultrasonic sound have been transmitted, and the handler will be called again when the ECHO signal goes low, whether from receiving an echo from a detected obstacle or because the 38 millisecond clock has elapsed.
   
2. Using a state machine, iterate over these sequential states:
   1. PULSE_HIGH \
      Set the trigger signal high.
   2. PULSE_LOW \
      Set the trigger signal low.
   3. COUNTDOWN \
      Wait for the echo signal. 
     
# Managing Timing

There are three kinds of timing going on.

1. Loop timing. \
   The Arduino runs a ***loop*** function continuously. This will only be used to look for special handling driven by the detected obstacle distances. For example, you might want to raise an alarm of some sort if the distance is less than some critical distance. Each time through the loop, you can look at the last cached distance value for every SONAR device and respond accordingly.
2. Periodic timer. \
   A timer is setup so that at a fixed rate, say every 10 milliseconds, a timer handler is called. This is used to drive a state machine which starts out by setting the ***TRIGGER*** signal high and, during when the handler is called 10 milliseconds later, the ***TRIGGER*** pulse is set low. Then, a countdown period begins which is used to provided a fixed length of time for handling each SONAR device. At the end of the countdown period, a ***device index*** is incremented and the state machine goes back to the state when at the next handler call will set ***TRIGGER***  high again, but for the next SONAR device.
3. Echo time measurement. \
   Shortly after the ***TRIGGER*** signal is set low by the state machine, the hardware device will set the ***ECHO*** signal high. This will cause the interrupt handler for the SONAR device to be invoked. The interrupt handler, seeing a low to high signal change, will capture the current time, in micro seconds. Then, when either the actual ultrasonic echo sound is received or the 38 millisecond timer built into the device times out, the ***ECHO*** signal will go low and call the interrupt handler for the device again. The handler, seeing a high to low signal change, will compute the duration, in microseconds, from the time when the ***ECHO*** signal last went high, multiply that time by the speed of  in meters per second, and store the resulting distance measurement as a cached value for the device. That cached value can then be used, for example in the loop timing described above, to check for some safety condition, or can be read at any time and reported as the last known distance detected by the SONAR device.

The management of the SONAR devices is all timer and interrupt driven. A list of SONAR devices to be managed is set up. During each trigger of the timer, say every 10 micro seconds, the currently managed SONAR device is either setting the ***TRIGGER*** signal high, setting the ***TRIGGER*** signal low, or waiting out the configured period before advancing to the next SONAR device in the list. This is all driven by a periodic timer.

Meanwhile, the handling of the ***ECHO*** signal for each SONAR device is done with an interrupt handler. Each SONAR device has it's own interrupt handler. Some time after the ***TRIGGER*** signal is set low, the ***ECHO*** signal will go high and the current time is captured. When the ***ECHO*** signal goes low again, the duration is divided in half and multiplied by the speed of sound. Remember, the signal has to go out, hit an obstacle, and reflect back, so the distance to the obstacle is half the duration of the ***ECHO*** response.

In the case of a ROS 2 sensor handler, each SONAR needs to periodically report back what the detected obstacle distance for each sensor. That can be done as part of the ***loop*** function. Inside the ***loop*** cycle handling, the time of the last sensor report is captured, and when enough time since then has passed that the next report is needed, you just capture the last known distance cached for the SONAR device and report that. It's up to you to decide to send the list of all distances in a single report, or to send the reported distance of one sensor at a time, much like the distances are captured one at a time in a round robin manner.