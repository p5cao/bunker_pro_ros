#!/usr/bin/env python
from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

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

msg = """
Reading from the left and right sticks  and Publishing to Twist!
---------------------------
left stick drive forward/backward
right stick turn left/right


anything else : stop

Y/A : increase/decrease max speeds by 10%

w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self._twist = Twist()
        self.publisher = rospy.Publisher('/cmd_vel', TwistMsg, queue_size = 5)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.joy = Joy()
        self._timer = rospy.Timer(rospy.Duration(0.05), self.joy_controller)
        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        self._deadman_pressed = False
        self._zero_twist_published = False
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def joy_callback(self, data):
        """ Receive joystick data, formulate Twist message. """
        self.joy = data
        # cmd = TwistMsg()
        self.x = data.axes[1]
        self.y = 0
        self.z = 0
        self.th = data.axes[3]
        # print("joy left stick:", data.axes[1])
        # print("joy right stick:", data.axes[3])
        self._deadman_pressed = data.buttons[4] or data.buttons[5]

    def joy_controller(self, *args):
        self.publisher.publish(self._twist)
        if self._deadman_pressed:
            self._zero_twist_published = False
        elif not self._zero_twist_published:
            self._zero_twist_published = True

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            rospy.Subscriber("/joy_orig", Joy, self.joy_callback)
            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn
            #print("cmd_vel x:", twist.linear.x)
            #print("cmd_vel theta:", twist.angular.z)

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)



def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_joy')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.5)

    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))

        while(1):
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
            print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)         

# def vels(speed, turn):
#     return "currently:\tspeed %s\tturn %s " % (speed,turn)

# class JoyDrive:
#     def __init__(self):
#         rospy.init_node('joy_drive')

#         self.turn_scale = rospy.get_param('~turn_scale', 0.5)
#         self.drive_scale = rospy.get_param('~drive_scale', 0.5)
#         self.deadman_button = rospy.get_param('~deadman_button', 0)
#         # self.tag_timeout_duration = rospy.get_param('~tag_timeout', 5)
        
#         self.tag_timeout_time = rospy.get_rostime()
        
#         self.cmd = None
#         self.joy = Joy()
#         cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)

#         rospy.Subscriber('/joy', Joy, self.joy_callback)
        
#         rate = rospy.Rate(rospy.get_param('~hz', 20))
        
#         while not rospy.is_shutdown():
#             rate.sleep()
#             if self.cmd:
#                 cmd_pub.publish(self.cmd)


#     def joy_callback(self, data):
#         """ Receive joystick data, formulate Twist message.
#         Use planner if a secondary button is pressed """
#         self.joy = data
#         cmd = Twist()
#         cmd.linear.x = data.axes[1] * self.drive_scale
#         cmd.angular.z = data.axes[3] * self.turn_scale

#         # if data.buttons[self.deadman_button] == 1 and
#         #   self.tag_timeout_time - rospy.get_rostime() > 0:
#         #     self.cmd = cmd
#         # else:
#         #     self.cmd = None

# if __name__ == "__main__": 
#     print(msg)
#     JD = JoyDrive()
#     speed = JD.drive_scale
#     turn = JD.turn_scale
#     print(vels(speed,turn))
    