#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import numpy as np

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

TwistMsg = Twist
data = Joy

msg = """
Reading from the left and right sticks and Publishing twist to /cmd_vel!
---------------------------
left stick drive forward/backward
right stick turn left/right


anything else : stop

Y/A : increase/decrease max speeds by 10%

w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

class JoyControlDamBot:

    def __init__(self):
        # Create a ROS publisher
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Initialize TwistMsg data
        self.throttle_stick = 0.0
        self.turn_stick = 0.0
        self.throttle_data = []
        self.turn_data = []

        self.reading_index = 0
        self.reading_max_index = 10

        self.throttle_filterd = 0.0
        self.turn_filterd = 0.0

        self.throttle_interpolated = 0.0
        self.turn_interpolated = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.speed = 1.0
        self.turn = 0.5
        self.lin_vel_increase = 0.0
        self.lin_vel_decrease = 0.0
        self.ang_vel_increase = 0.0
        self.ang_vel_decrease = 0.0
        self.twist_original = TwistMsg()
        self.twist_interpolated = TwistMsg()

    def joy_callback(self, data):
        
        self.joy = data
        # cmd = TwistMsg()
        self.throttle_stick = data.axes[1]
        self.turn_stick = data.axes[3]
        self.lin_vel_increase = data.buttons[3]
        self.lin_vel_decrease = data.buttons[0]
        self.ang_vel_decrease = data.buttons[2]
        self.ang_vel_increase = data.buttons[1]
        # print("joy left stick:", data.axes[1])
        # print("joy right stick:", data.axes[3])
        # self._deadman_pressed = data.buttons[4] or data.buttons[5]

    def read_gamepad_data(self, event = None):
        # Here you read the data from your joy_node and gamepad
        # /joy_orig publishes at 200Hz
        """ Receive joystick data, formulate Twist message. """
        rospy.Subscriber("/joy_orig", Joy, self.joy_callback)

        # oversampling and averaging
        if self.reading_index < self.reading_max_index:
            self.throttle_data.append(self.throttle_stick)
            self.turn_data.append(self.turn_stick)
            self.reading_index += 1
        else:
            self.throttle_filterd = sum(self.throttle_data) / len(self.throttle_data)
            self.turn_filterd = sum(self.turn_data) / len(self.turn_data)
            self.reading_index = 0
            self.throttle_data = []
            self.turn_data = []

    def throttle_piecewise(self, throttle_filterd):
        tf = throttle_filterd
        condlist = [(tf <-0.9), # -1.0
                    (tf >=-0.9 and tf <-0.7), # -0.8
                    (tf >=-0.7 and tf <-0.5), # -0.6
                    (tf >=-0.5 and tf <-0.3), # -0.4
                    (tf >=-0.3 and tf <-0.1), # -0.2
                    (tf >=-0.1 and tf <=-0.1), # 0.0
                    (tf > 0.1 and tf <= 0.3), # 0.2
                    (tf > 0.3 and tf <= 0.5), # 0.4
                    (tf > 0.5 and tf <= 0.7), # 0.6
                    (tf > 0.7 and tf <= 0.9), # 0.8
                    (tf > 0.9 ), # 1.0
                    ]
        funclist = [-1.0, -0.8, -0.6, -0.4, -0.2, 0.0,
                     0.2,  0.4,  0.6,  0.8,  1.0]
        throttle = np.piecewise(tf, condlist, funclist)
        return throttle

    def publish_cmd_vel(self, event=None):
        #twist = TwistMsg()
        self.twist_interpolated.linear.x = self.throttle_piecewise(self.throttle_filterd)*self.speed
        self.twist_interpolated.linear.y = 0.0
        self.twist_interpolated.linear.z = 0.0
        self.twist_interpolated.angular.x = 0.0
        self.twist_interpolated.angular.y = 0.0
        self.twist_interpolated.angular.z = self.throttle_piecewise(self.turn_filterd)*self.turn

        self.cmd_vel_publisher.publish(self.twist_interpolated)

    def vels(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed,self.turn)

if __name__ == '__main__':
    rospy.init_node("teleop_twist_joy")
    pt = JoyControlDamBot()

    try:
        print(msg)
        print(pt.vels())
        # Create a ROS Timer for reading data
        rospy.Timer(rospy.Duration(1.0/50.0), pt.read_gamepad_data)

        # Create another ROS Timer for publishing data
        rospy.Timer(rospy.Duration(1.0/50.0), pt.publish_cmd_vel)

        rospy.spin()

    except Exception as e:
        print(e)
