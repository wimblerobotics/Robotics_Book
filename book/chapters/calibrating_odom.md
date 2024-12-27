# Calibrating your odometry

## Why calibrate your odometry?

The odometry of a robot is the measure of the robot's position and orientation in space.
It is usually calculated by integrating the robot's velocity over time.
This happens by sampling the rotation of the wheels as sensed by the wheel encoders.
Knowing the rotation of each wheel in radians, the wheel radius and the time between samples,
you can calculate the distance traveled and the velocity of each wheel during
that sample interval.
Knowing the distance between the wheels, you can calculate the arc or line the robot traveled during the sample interval.
By integrating these arcs or lines, you can calculate the robot's position and orientation over time relative to where the robot was when it started up.

But wheel odometry is not perfect.
There are errors in the measurements of the wheel encoders, such as noisy signals which may
undercount or overcount the 'ticks' from the wheel encoders.
The smapling interval is not precisely known.
There are errors in the math used for the computation of the integration of the velocity.
These errors can accumulate over time and cause the ocometry to drift away from its actual
position and orientation.

If the errors are too large, or the odometry reports don't come in fast enough, the ability to create a map of the environment or to navigate through it can be compromised.

Odometry as described, relies especially on two fundamental physical measurements of the robot:
the wheel radius and the distance between the wheels.
The wheel radius is usually the easier to estimate with fair precision using, say, a caliper.
But that won't be enough.
The effective wheel radius may change as the tire wears, or as the wheel tilts,
or depend on the kind of surface the robot is moving on.
Even knowing the wheel radius, the odometry may still be off if the wheel radius is not the same for both wheels.
Robot wheels often slip, and usually in an unpredictable way.

The distance between the wheels is harder to estimate, and it is usually the source of the largest errors in odometry.
The distance between the wheels can be estimated by measuring the distance between the wheel centers,
but this is not always accurate, and it can vary as the tires wear, or the wheels themselves tilt.
Tires are usually not flat and the point of contact with the ground may change for a number of reasons.
The distance may be different when the robot is moving in a straight line, or it may be different when making
a left turn from than when making a right turn.

The general result is one of the reasons that I often say that everything about robots is hard.
My example when I give talks about robots is that just because the robot's computer tells the motor controller to turn the wheels
doesn't mean that the controller got that command.
If the motor controller got a command to move the wheels, it doesn't mean that the motors actually turned.
If the motors turned, it doesn't mean that the wheels turned.
If the wheels turned, it doesn't mean that the robot moved.
If the robot moved, I can usually say that the robot did not go exactly where is was supposed to go.

It's part of the mantra that sensors lie all the time, and it's the job of the software to figure
out how much they are lying and correct for it.
Forturnately, figuring out how much the odometry is lying is somewhat tractable as localization
is usually a part of a closed loop system.
The software never just assumes that the robot moved as commanded, it also checks sensors like LIDARs,
cameras, time of flight sensors, SONAR, IMUs, GPS and so on to try to correct the odometry.

Still, the more accurate the odometry, the better the robot can navigate and map. Converserly,
if the odometry is very far off, the robot may not be able to navigate or map at all.

Calibrating the odometry for the purposes of this chapter, is the process of trying to improved the measurement
of the wheel diameter and the distance between the wheels.
The process is usually done by driving the robot in a controlled way and comparing the odometry
to the actual position and orientation of the robot.

## How to calibrate your odometry

I usually separate the calibration of the two measurements.
The process is fundamentally the same for correcting the wheel radius and the distance between the wheels,
but correcting the wheel radius usually involves different equipment (well, a compass versus a scale)
and a lot more iterations of the process.

The steps involved in both calibrations are, basically:

* Place the robot in a known position and orientation.
* Read the odometry at the beginning of the iteration.
* Move the robot in a known way that should be affected primarily by just the
wheel radius or the distance between the wheels.
* Read the odometry at the end of the iteration.
* Compare the actual position and orientation of the robot to the odometry.
* Adjust the wheel radius or the distance between the wheels.
* Repeat until the odometry is as close as possible to the actual position and orientation.

The assumption here is that you have the ability to alter the input to the process that is
producing the odometry. Odometry comes in two forms, as a topic and as a transform.
If the some manufacturer's driver is producing the odometry, you should be able to alter the
parameters of the driver to get the odometry to be more accurate.
Often this comes in the form of a configuration file that you can edit, or is supplied as a parameter
in a launch file.
If you are writing your own odometry, as in writing your own motor driver software or writing, say,
a class that implements the ros_control API, you should be able to alter the code to use the calibrated
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
odomentry using a Linux process, you will be hit with process preemption and the like which is going to
make it hard to get good odometry. If at all possible, never computer odometry using a computer that is
going to result in the odometry code not being called often or not being called at a regular interval.
Even if you are using, say, an Arduino processor, make sure you understand now to make your loops
run at a regular interval and that any interrupt handlers don't affect system timing.

## Calibrating the wheel radius

Repeat these steps until odometry measured from driving the robot in a straight line is pretty close between
the observed physical position of the robot and the reported odometry. You need to do this measurement
repeatedly. Don't assume that when it works once that you're done.
Try it several times, maybe a dozen times. 
You may even find that odometry is accurate when, say, driving forward but is off when driving backwards.
If that's your case, you will need to be creative in how you drive the robot to get the odometry to be accurate
as this problem won't just affect driving in a straight line. If at all possible, fix your robot so that
odomentry is accurate whether driving forward or backward.

