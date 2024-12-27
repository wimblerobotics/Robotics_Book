# Calibrating your odometry

## Why calibrate your odometry?

The odometry of a robot is the measure of the robot's position and orientation in space.
It is usually calculated by integrating the robot's velocity over time.
This happens by sampling the rotation of the wheels as sensed by the wheel encoders.
Knowing the rotation of each wheel in radians, the wheel radius and the time between samples,
you can calculate the distance traveled and the velocity of each wheel during
that sample interval.
By knowing the distance between the wheels, you can calculate the arc or line the robot traveled during the sample interval.
By integrating these arcs or lines, you can calculate the robot's position and orientation over time relative to where the robot was when it started up.

Note that correct integration relies on integration happening over short distances.
If the robot moves too far between samples, the integration can be wildly off.
This means that the odometry calculation should occur often, on order of 20 to hundreds of times per second
for a robot moving at a walking pace. More times per second is better as a rule of thumb.

But wheel odometry is not perfect.
There are errors in the measurements of the wheel encoders, such as noisy signals which may
undercount or overcount the 'ticks' from the wheel encoders.
The sampling interval is not precisely known.
There are errors in the math used for the computation of the integration of the velocity.
These errors can accumulate over time and cause the odometry to drift away from its actual
position and orientation.

If the errors are too large, or the odometry reports don't come in fast enough, the ability to create a map of the environment or to navigate through it can be compromised.

Odometry as described, relies especially on two fundamental physical measurements of the robot:
the wheel radius and the distance between the wheels.
The wheel radius is usually the easier to estimate with fair precision using, say, a caliper.
But that won't be enough.
The effective wheel radius may change as the tire wears, or as the wheel tilts,
or depend on the kind of surface the robot is moving on.
Even knowing the wheel radius, the odometry may still be off if the wheel radius is not the same for both wheels.
Robot wheels often slip on the ground as they turn, and usually in an unpredictable way.

The distance between the wheels is harder to estimate, and it is usually the source of the largest errors in odometry.
The distance between the wheels can be estimated by measuring the distance between the wheel centers,
but this is not always accurate and is affected by several factors.
Tires are usually not flat and the point of contact with the ground may change for a number of reasons.
The distance may be different when the robot is moving in a straight line, or it may be different when making
a left turn from than when making a right turn. The wheels may be tilted, and each wheel can be tilted differently.
The wheels might wobble, making the distance between them change as the wheels turn.

The general result is one of the reasons that I often say that everything about robots is hard.
My example when I give talks about robots is that just because the robot's computer tells the motor controller to turn the wheels
doesn't mean that the controller got that command.
If the motor controller got a command to move the wheels and sent a signal for the motors to turn,
it doesn't mean that the motors actually turned.
If the motors turned, it doesn't mean that the wheels turned.
If the wheels turned, it doesn't mean that the robot moved.
If the robot moved, I can usually say that the robot did not go exactly where is was supposed to go.

It's part of the mantra that sensors lie all the time, and it's the job of the software to figure
out how much they are lying and correct for it.
Forturnately, figuring out how much the odometry is lying is somewhat tractable as localization
is usually a part of a closed loop system.
The software never just assumes that the robot moved as commanded, it also checks sensors like LIDARs,
cameras, time of flight sensors, SONAR, IMUs, GPS and so on to try to correct the oreported dometry to
agree with other sensors.

Still, the more accurate the odometry, the better the robot can navigate and map. Converserly,
if the odometry is very far off, the robot may not be able to navigate or map at all.

Calibrating the odometry for the purposes of this chapter, is the process of trying to improve the measurement
of the wheel diameter and the distance between the wheels.
The process is usually done by driving the robot in a controlled way and comparing the odometry
to the actual position and orientation of the robot.

The following methodology assumes you are calibrating a two-wheel, differential drive robot.
You should be able to generalize this process for a different kind of robot.

## How to calibrate your odometry

I usually separate the calibration of the two measurements.
The process is fundamentally the same for correcting the wheel radius and the distance between the wheels,
but correcting the wheel radius usually involves different equipment (well, a compass versus a scale)
and a lot more iterations of the process.

The steps involved in both calibrations are, basically:

* Place the robot in a known position and orientation.
* Read the odometry at the beginning of the iteration.
* Move the robot in a known way that should be affected primarily by just one of the
wheel radius or the distance between the wheels.
* Read the odometry at the end of the iteration.
* Compare the actual position and orientation of the robot to the odometry.
* Adjust the software values for the wheel radius or the distance between the wheels.
* Repeat until the odometry is as close as possible to the actual position and orientation.

The assumption here is that you have the ability to alter the input to the process that is
producing the odometry. Odometry comes in two forms, as a topic and as a transform.
If some manufacturer's software driver is producing the odometry, you should be able to alter the
parameters of the driver to get the odometry to be more accurate.
Often this comes in the form of a configuration file that you can edit, or is supplied as a parameter
in a launch file.
If you are writing your own odometry, as in writing your own motor driver software or writing, say,
a class that implements the ros2_control API, you should be able to alter the code to use the calibrated
wheel radius and distance between wheels.

Also note that these two measurements are likely to not be exactly the same as that used to create the URDF
for the robot. The URDF is mostly used for visualization and collision detection, and the measurements
in the URDF are for visual purposes.
You can used the actual, calibrated measurements in the URDF, but you can be less accrute in the URDF,
though it's possible that you pass the calibration values via the URDF to the odometry code.
I do not. I have a separate configuration file for the calibrated measurements because those measurements
will change over time, especally as the robot is used and parts wear.

Now, pay attemption to this. Remember all of those things mentioned above that can affect
the odometry. The sampling interval isn't precise, wheels slip, and so on. Especially if you are computing
odomentry using a Linux process, you will be hit with process preemption and similar Linux issues which are going to
make it hard to get good odometry. If at all possible, never compute odometry using a computer that is
going to result in the odometry code not being called often or not being called at a regular interval.
Even if you are using, say, an Arduino processor, make sure you understand now to make your loops
run at a regular interval and that any interrupt handlers don't affect system timing.

## Calibrating the wheel radius

Repeat the following steps until odometry measured from driving the robot in a straight line is pretty close between
the observed physical position of the robot and the reported odometry. You need to do this measurement
repeatedly. Don't assume that when it works once that you're done.
Try it several times, maybe a dozen times.
You may even find that odometry is accurate when, say, driving forward but is off when driving backwards.
If that's your case, you will need to be creative in how you fix the robot to get the odometry to be accurate
since this problem won't just affect driving in a straight line. If at all possible, fix your robot so that
odomentry is accurate whether driving forward or backward.

1. Place the robot in a known position and orientation.
Before you move the robot, mark its current position.
The easy way to do this is to take a short length of blue, painter's tape and put it on the floor
at the very bottom of one of the wheels, where it touches the ground.
It doesn't have to be under the wheel, it could be beside the wheel.
After the robot moves, you will put another piece of tape at the bottom of the same wheel and measure
the distance, in meters, between the two pieces of tape.

1. Read the current odometry value via the following command. Replace the ***/odom*** topic with the
topic that your odometry is published on.

   ```code
   ros2 topic echo --once --flow-style /odom
   ```

   You will see output something like:

   ```code
   header:
     stamp:
       sec: 16
       nanosec: 320000000
     frame_id: odom
   child_frame_id: base_link
   pose:
     pose:
       position:
         x: -3.40717210659393e-16
         y: -8.0459616687065e-36
         z: 0.0
       orientation:
         x: 0.0
         y: 0.0
         z: 5.0907089586332814e-20
         w: 1.0
     covariance:
   ```

   With more lines after the ***covariance*** line. There will be other differences as well.
   The information you want to capture is the ***x*** value under ***position***.
   This is the current position of the robot.
   You will next drive the robot forward a bit and, ideally, only the ***x*** value will change,
   becoming more positive.

1. Move the robot forward a known distance. The way I do this is to use the ***teleop_keyboard_twist**
package, like so:

   ```code
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

   You will see a prompt in the terminal window. Press the ***i*** key to move the robot forward and then
   after the robot has moved forward a bit, ideallly at least a meter, press the ***k*** key to stop the robot.

1. As above, read the current odometry  value again and capture the ***x*** value under ***position***.
The new ***x*** value minus the previous value is the distance the software thought the robot moved,
mostly based on the wheel encoders.
You should have found that the other values under the ***position*** and even under ***orientation*** have not
changed much, if at all.
The ***x*** value now should be more positive than the ***x*** value you captured before moving the robot.

1. Measure the distance between the two pieces of tape. This is the actually distance the robot moved.
If the robot moved forward more than a meter, and the odom distance and the actual distance are nearly
identical, say within one or two millimeters, you are lucky, indeed.
The goal is two get the odom distances and the actual distances to be as close as possible repeatedly.
If this is so after, say, a dozen movements, forward and backward, you are done.

1. If the odom distance and the actual distance are not close, you will need to adjust the wheel radius
as known to your motor driver software.
If the actual distance is greater than the odom distance, you will need to increase the wheel radius.
Likewise, if the actual distance is less than the odom distance, you will need to decrease the wheel radius.
You could calculate the new wheel radius by multiplying the current wheel radius by the ratio of the actual
distance to the odom distance.
If that is too much math, you could just add or subtract a small amount to the current wheel radius value.

1. Repeat the process until the odom distance and the actual distance are as close as possible.
What you are likely to find is that as you change the wheel radius value, you will end up with
finding that the actual distance is sometimes a little bit bigger and sometimes a little bit smaller
than the odom distance as you repeat the test.
When you change the wheel radius to a larger or smaller value, the error in the two distances
will tend to be larger than with the old wheel radius value. This will likely be the best you can do
for calibrating the wheel radius.

## Calibrating the distance between the wheels

This process is similar to that of calibrating the wheel radius.
Repeat the following steps until the odometry measured from rotating the robot in place is pretty close between
the observed physical heading of the robot and the heading from the odometry topic.
You need to do this measurement repeatedly.
Don't assume that when it works once that you're done.
Try it several times, maybe a dozen times.
You may even find that odometry is accurate when, say, turning left but is off when turning right.

To make this process easier, I've provided a LibreCalc spreadsheet that will help you calculate the
error in rotation of the robot.
It can be found at [book/media/CalibratingRotation.ods](book/media/CalibratingRotation.ods).

Each time you are going to do a rotation, put the current distance between the wheels into the spreadsheet
in column ***A***. This is just for documentation purposes.
To correct the distance between the wheels, you are going to observe the difference between the actual
rotation of the robot and the rotation reported by the odometry, make a change to the
configuration value of the distance between the wheels, do a 360 degree rotation again,
and then observe the difference again.
By keeping track of the distance between the wheels for each rotation test, you can see if you should be
making the configuration distance between the wheels larger or smaller to get a smaller error in the actual versus
the expected rotation.

Here are the steps.

1. Place the robot in a known position and orientation.
Before you move the robot, mark its current position.
The easy way to do this is to use your phone with a built-in compass app and place the phone on the surface of your robot and don't move it until you are done calibrating, or to mark the robot's
current position with a piece of tape by putting it on the ground beside one of the wheels
at the bottom of the wheel where it touches the ground.
You are going to rotate the robot in place and you want to try to rotate the robot 360 degrees
in place, getting the marked wheel back to the original position.

1. Read the current odometry value via the following command.
  Replace the ***/odom*** topic with the topic that your odometry is published on.

   ```code
   ros2 topic echo --once --flow-style /odom
   ```

   You will see output something like:

   ```code
   header:
     stamp:
       sec: 16
       nanosec: 320000000
     frame_id: odom
   child_frame_id: base_link
   pose:
     pose:
       position:
         x: -3.40717210659393e-16
         y: -8.0459616687065e-36
         z: 0.0
       orientation:
         x: 0.0
         y: 0.0
         z: 5.0907089586332814e-20
         w: 1.0
     covariance:
   ```

   With more lines after the ***covariance*** line. There will be other differences as well.
   The information you want to capture is the ***w*** and ***z***  values under ***orientation***.
   This is the current position of the robot.
   
   If this is the beginning of the calibration, you should place the ***w*** and ***z*** values into
   the spreadsheet on line 2.
   As mentioned above, you should always note the current distance between the wheels in column ***A***.
   You will next rotate the robot in place and, ideally, only the ***w*** and ***z*** values will change noticeably.

   If you are using a compass to track rotation, you can use the ***G*** column to note the compass heading.

1. Rotate the robot in place 360 degrees. The way I do this is to use the ***teleop_keyboard_twist** package, like so:

   ```code
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

   You will see a prompt in the terminal window. Press the ***l*** key to rotate the robot clockwise in place
   or the ***j*** key to rotate anticlockwise and then after the robot has rotated 360 degrees, press the
   ***k*** key to stop the robot.
   I found that I needed to jog the robot a bit to get to an accurate 360 degree rotation.
   This I did by pressing the ***j*** or ***l*** key for a short time and then pressing the ***k***
   key to stop the robot.
   Remember you are trying to get to the point where the compass is back to the same value as when you started the 360 degree rotation,
   if you are using a compass to measure rotation, or to where the bottom of the wheel is back to the same
  position as when you started.

1. As above, read the current odometry value again and capture the ***w*** and ***z*** values under ***orientation***.
   If your robot is a well designed, 2-wheel differential drive robot, the ***x*** and ***y*** values under ***position**
    should not have changed much from the values captured in step 2, nor should the ***x*** and ***y*** values under ***orientation***.

1. Using the spreadsheet that I provided put the original values of ***w***
   and ***z*** from step 2 into the spreadsheet
   into the next row and it will calculate the rotation reported by the odom topic.
   Column ***D*** shows the reported heading of the robot from the ***odom*** topic.
   Column ***E*** shows the reported rotation in radians of the robot from the ***odom*** topic.
   Column ***F*** shows reported rotation in degrees. This value should be close to 360 or 0.

1. If the deviation in degrees in column ***F*** is greater than 360 degrees (i.e. a positive value), you will need
   to increase the configured distance between the wheels.
   If the deviation in degrees in column ***F*** is less than 360 degrees (i.e. a negative value), you will need
  to decrease the configured distance between the wheels.
   You could calculate the new distance between the wheels by multiplying the current distance between the wheels by the ratio of the actual
   rotation to the odom rotation.
   If that is too much math, you could just add or subtract a small amount to the current distance between the wheels value.

1. Repeat the process until the odom rotation and the actual rotation are as close as possible.
    What you are likely to find is that as you change the distance between the wheels value, you will end up with
    finding that the actual rotation is sometimes a little bit bigger and sometimes a little bit smaller
    than the odom rotation as you repeat the test.
    When you change the distance between the wheels to a larger or smaller value, the error in the two rotations
    will tend to be larger than with the old distance between the wheels value. This will likely be the best you can do
    for calibrating the distance between the wheels.