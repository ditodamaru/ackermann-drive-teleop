#!/usr/bin/env python

'''
ackermann_drive_joyop.py:
    A ros joystick teleoperation script for ackermann steering based robots
'''

'''
test
'''

__author__ = 'George Kouros'
__license__ = 'GPLv3'
__maintainer__ = 'George Kouros'
__email__ = 'gkourosg@yahoo.gr'

import rospy
from ackermann_msgs.msg import AckermannDriveStamped #AckermannDrive
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
import sys

class AckermannDriveJoyop:

    def __init__(self, args):
        if len(args)==1 or len(args)==2:
            self.max_speed = float(args[0])
            self.max_steering_angle = float(args[len(args)-1])
            cmd_topic = 'ackermann_cmd'
        elif len(args) == 3:
            self.max_speed = float(args[0])
            self.max_steering_angle = float(args[1])
            cmd_topic = '/' + args[2]
        else:
            self.max_speed = 0.58  #0.2
            self.max_steering_angle = 0.39 #0.44 #0.7
            cmd_topic = 'ackermann_cmd'

        self.speed = 0
        self.steering_angle = 0
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.control_state_sub = rospy.Subscriber('/joy_control_state', Int32, self.control_state_callback)
        self.connection_state_sub = rospy.Subscriber('/joy_connection_error', Int32, self.connection_state_callback)        
        self.drive_pub = rospy.Publisher(cmd_topic, AckermannDriveStamped,
                                         queue_size=1) #AckermannDriveStamped
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        rospy.loginfo('ackermann_drive_joyop_node initialized')

    def joy_callback(self, joy_msg):
        self.speed = joy_msg.axes[2] * self.max_speed#;
        self.steering_angle = joy_msg.axes[3] * self.max_steering_angle#;

    # joy_control_state
    def control_state_callback(self, control_state_msg):
        self.control_state = control_state_msg.data

    # joy_connection_error
    def connection_state_callback(self, connection_state_msg):
        self.connection_state = connection_state_msg.data
         # block for handle Joy connection lost
        if self.connection_state == 0: # if joystick connection lost
            self.speed = 0
            self.steering_angle = 0
            
            #Publish zero values to stop the robot
            ackermann_cmd_msg = AckermannDriveStamped()
            ackermann_cmd_msg.drive.speed = self.speed
            ackermann_cmd_msg.drive.steering_angle = self.steering_angle
            self.drive_pub.publish(ackermann_cmd_msg)
            rospy.loginfo("Joystick connection lost. Stopping the robot.")
    

    def pub_callback(self, event):
        #add line for logic
        if self.control_state == 3:  #if control_state button pressed
            ackermann_cmd_msg = AckermannDriveStamped() #AckermanDrive
            ackermann_cmd_msg.drive.speed = self.speed
            ackermann_cmd_msg.drive.steering_angle = self.steering_angle
            self.drive_pub.publish(ackermann_cmd_msg)
            self.print_state()

        # line for handle standby state
        #if self.control_state == 4: #if standby state
        else:
            # Reset speed and steering angle to zero
            self.speed = 0
            self.steering_angle = 0

            # Publish zero values to stop the robot
            ackermann_cmd_msg = AckermannDriveStamped()
            ackermann_cmd_msg.drive.speed = self.speed
            ackermann_cmd_msg.drive.steering_angle = self.steering_angle
            self.drive_pub.publish(ackermann_cmd_msg)
            rospy.loginfo("Control state released or on standby state. Stopping the robot.")
    

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.drive_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_joyop_node')
    joyop = AckermannDriveJoyop(sys.argv[1:len(sys.argv)])
    rospy.spin()
