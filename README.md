# Improvement
- [x] add safety features
- [x] refine the script, the terminal output not good

# ackermann_drive_teleop
ROS teleoperation scripts for robots with ackermann steering

##### ackermann_drive_keyop
+ Run the teleoperation script, with  
`rosrun ackermann_drive_teleop keyop.py`  
+ You can set max speed, steering angle and command topic by giving them as arguments, when running the script.  
eg.1 `rosrun ackermann_drive_teleop keyop.py 0.5`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; -> max_speed=max_steering_angle=0.5, command_topic=/ackermann_cmd  
eg.2 `rosrun ackermann_drive_teleop keyop.py 0.5 0.8`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ->  max_speed=0.5, max_steering_angle=0.8, command_topic=/ackermann_cmd  
eg.3 `rosrun ackermann_drive_teleop keyop.py 0.5 0.8 ack_cmd`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ->  max=speed=0.5, max_steering_angle=0.8, command_topic=/ack_cmd  
+ Use the "up", "down" arrow keys to control speed, "left" and "right" arrow keys to control the steering angle,  
  space to brake and tab to reset the steering angle.  

##### ackermann_drive_joyop
+ Run the teleoperation script, as well as the joy node using the following command:  
`roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch`  
+ You can set max speed and steering angle, by giving them as arguments to the launcher.  
eg. `roslaunch ackermann_drive_teleop ackermann_drive_joyop.launch max_speed:=0.5 max_angle:=0.8`  
+ **In order to use a joystick, it must have read and write permissions.**  
You can grant such permissions by executing the following command: `sudo chmod a+rw /dev/input/js0`

### Usage for HADA Robotic Platform

This is an example of how to operate HADA robot platform using joystick teleoperation
* Bringup the robot
  ```sh
    sudo ip link set can0 up type can bitrate 125000
  ```
* Terminal 2 : Launch Joystick Remapper (For MX Swicth Nintendo Joystick)
  ```sh
  roslaunch joystick_remapper joystick_remapper_ps3_hadarobot.launch
  ```
* Terminal 3 : Launch ackerman drive joystick teleoperation
  ```sh
  rosrun ackerman_drive_teleop joyup.py
  ```
* Terminal 4: Launch Cansend Generator
  ```sh
  rosrun cansend_generator cansend_generator.py
  ```
