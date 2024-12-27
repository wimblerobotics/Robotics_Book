* Put a new value for wheel_separation into the motor driver.
* Put my iPhone on the robot to measure degrees. I start at south, or 179 degrees.
* Compile and upload the code.
* Use ros2 topic echo /odom to see the values of the odometry. In my case, z is reset to 0 and w is reset to 1.**
* Use teleop_keyboard_twist to move the robot 180 degrees. Always using the same rotation direction (clockwise in my case) each time.
* Use https://quaternions.online/ and input the w and z values, apply rotation and see the Euler angle for z.
* Use the error to adjust the wheel_separation value.
* If you rotate 360 more than once, you reduce the measurement error.

```
ros@amdc:~$ ros2 topic echo -f /teensy_stats 
data: '{"Stats": {"loops":136,"Ms":1004.2,"mdls":[{"n":"Batt","MnMxAv":[0.0,0.2,0.0]},{"n":"uRos","MnMxAv":[0.0,4.0,0.4]},{"n":"Rlay","MnMxAv":[0.0,0.0,0.0]},{"n":"Robo","MnMxAv":[6.0,7.8,7.0]},{"n":"TSd","MnMxAv":[0.0,0.0,0.0]}]}}'

-- after 5th rotation
w: 0.94119453, z: 0.33786514 , 2.452ra, 39.5deg short, 140.5 pos (s.b. 180)
ros2 topic echo -f /roboclaw_status
data: '{"LogicVoltage":0.0,"MainVoltage":0.0,"Encoder_Left":174924,"Encoder_Right":-41865,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":0}'

--- after 6th rotation
x: -0.0005368524580262601 y: 0.0010069749550893903
w: -0.8901238441467285 z: 0.4557186961174011
rad: -2.195, deg 305.78, -125.77 short
2.452-2.195=0.257, 0.257*2*3.14159=1.61deg
data: '{"LogicVoltage":0.0,"MainVoltage":0.0,"Encoder_Left":178800,"Encoder_Right":-45742,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":0}'

--- after 7th rotation
x: -0.0004595511418301612 y: 0.0009262574603781104
z: -0.5196370482444763 w: 0.854387104511261
4.234 rad, -62.61 deg, 117.39 short
-2.195-4.234=-6.429, ((2*pi)-6.429)*2*3.14159=-40.3deg
data: '{"LogicVoltage":0.0,"MainVoltage":0.0,"Encoder_Left":182658,"Encoder_Right":-49600,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":0}'

```

0.4	0.99135	-0.13123			-15.081	3.40481238125433	-15.081379479761	195.081379479761							
  -0.98146456	0.19164381			-22.097	-2.75591920448517	337.902539096052	-157.902539096052							
0.401	-0.997742	0.06715119			-7.701	-3.00718902469459	352.299239313062	-172.299239313062							
0.397	-0.997411	0.071903			-8.247	-2.99766236006168	351.75340163676	-171.75340163676							
  0.990331	-0.1387221			-15.948	3.41993459318726	-15.94781840042	195.94781840042							
0.405	0.08420584	0.08420584				1.5707963267949	90	90		2 rotations					
  0.987375	0.158399				2.82345465379234	18.2279646911281	161.772035308872		2nd 2 rotations					
  0.999986648	-0.0051648				3.15192229966234	-0.59184512382069	180.591845123821		-2 rotations					
  0.988662958	0.1501518				2.84014903136851	17.2714473144154	162.728552685585		-2 more rotations					
  0.951562762	0.3074545				2.51655904238133	35.8117879760658	144.188212023934		-2 more rotations					
0.41	0.989252	-0.146217				3.43507897714543	-16.8155276845491	196.815527684549		2 rotations					
0.4	0.99098	-0.1340096				3.41042062274787	-15.4027080478311	195.402708047831		2 rotations					
0.39	-0.9973	0.073101				-2.99525653986133	351.615558293013	-171.615558293013		2 rotations					
0.395	0.99010747	-0.14031098				3.42314364705388	-16.1316836432079	196.131683643208		2 rotations					
  0.99155747	-0.1296681				3.40166114220628	-14.900826782071	194.900826782071		reload and then 2 rotations					
0.393	0.99155	-0.129657				3.40164106669492	-14.8996765399985	194.899676539999		2 rotations					
0.391	0.980139	-0.198307				3.54085366015203	-22.8759706001611	202.875970600161							
  0.9920145	-0.1261231				3.39451245951619	-14.4912374348501	194.49123743485		reload and then 2 rotations					
  0.9681016	-0.25055				3.64809035489982	-29.0201806181422	209.020180618142		Total 4 rotations					
0.38	-0.997295	0.073496207				-2.99446750924463	351.570350168769	-171.570350168769		1	-8.5				
  0.9912975	-0.1316404342				3.40563992994125	-15.1287945268631	195.128794526863		2	15				
  0.990107953548	-0.1403076				3.42313681823305	-16.1312923805949	196.131292380595			16	rock back and forth back to 180			
  -0.9798275	0.1998447				-2.73919377779712	336.944242736271	-156.944242736271		3	-23	big pause during rotation			
  0.964629411	-0.26360973				3.67511743645521	-30.5687183238218	210.568718323822		4	30		ENC LEFT	ENC RIGHT	
  0.94119453	0.3378651142				2.45229724846003	39.4937175516954	140.506282448305		5	-40		171062,	-38003,	data: '{"LogicVoltage":0.0,"MainVoltage":0.0,"Encoder_Left":171062,"Encoder_Right":-38003,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":0}'
  -0.890123844146729	0.455718696117401				-2.19523372985155	305.777627765256	-125.777627765256		6					
  0.854387104511261	-0.519637048244476				4.23444483454031	-62.6158176001317	242.615817600132		7					

