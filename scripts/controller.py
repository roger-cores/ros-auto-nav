#!/usr/bin/env python
import rospy
import time
import math
from std_srvs.srv import Empty
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Joy
from tf.msg import tfMessage
from tf.transformations import *
import random

distance = 10
ranges = None
def distance_to_obs(scan):
    global distance
    global ranges
    distance = min(scan.ranges)
    ranges = scan.ranges
joystick_msg = None
def joystick_input(msg):
    global joystick_msg
    joystick_msg = msg


turtle_position = Twist()
def pose_changed(msg):
    turtle_position.linear.x = msg.twist.twist.linear.x
    turtle_position.linear.y = msg.twist.twist.linear.y
    turtle_position.linear.z = msg.twist.twist.linear.z

    turtle_position.angular.x = msg.twist.twist.angular.x
    turtle_position.angular.y = msg.twist.twist.angular.y
    turtle_position.angular.z = msg.twist.twist.angular.z

class TurtleCtrl:
    service_reset = rospy.ServiceProxy('reset', Empty)
    service_clear = rospy.ServiceProxy('clear', Empty)

    def __init__(self):
        self.location = Twist()
        self.location_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

    def publish_location(self):
        self.location_publisher.publish(self.location)

    @staticmethod
    def wait_for_services():
        rospy.wait_for_service('reset')
        rospy.wait_for_service('clear')

    def update_location(self, x, y, z, ax, ay, az):
        self.location.linear.x = x
        self.location.linear.y = y
        self.location.linear.z = z

        self.location.angular.x = ax
        self.location.angular.y = ay
        self.location.angular.z = az

    # rotate turtle bot
    # with speed (degrees per second)
    # and degrees (angular distance in degrees)
    # and direction (clockwise or anti-clockwise)
    def rotate(self, speed, degrees, clockwise):
        speed_r = math.radians(speed)
        radians = math.radians(degrees)

        if clockwise:
            self.update_location(0, 0, 0, 0, 0, -abs(speed_r))
        else:
            self.update_location(0, 0, 0, 0, 0, abs(speed_r))

        then = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < radians:
            self.publish_location()
            now = rospy.Time.now().to_sec()
            current_angle = speed_r*(now-then)

    def left(self, degrees, speed):
        self.rotate(speed, degrees, False)

    def right(self, degrees, speed):
        self.rotate(speed, degrees, True)

    def linear(self, distance, speed):
        covered = 0
        self.update_location(speed, 0, 0, 0, 0, 0)
        then = rospy.Time.now().to_sec()
        while covered < distance:
            self.publish_location()
            now = rospy.Time.now().to_sec()
            covered = abs(speed) * (now - then)
        self.stop()

    def stop(self):
        self.update_location(0,0,0,0,0,0)
        self.publish_location()

def init():
    global distance
    global ranges
    global joystick_msg
    rospy.init_node('controller', anonymous=True)
    time.sleep(2)
    rospy.Subscriber('/scan', LaserScan, distance_to_obs)
    rospy.Subscriber('odom', Odometry, pose_changed)
    turtle = TurtleCtrl()
    rospy.Subscriber('/joy', Joy, joystick_input)
    turtle.stop()
    time.sleep(5)
    while not rospy.is_shutdown():
        linear = 0.0
        angular = 0.0
        if joystick_msg is None:
            turtle.stop()
            time.sleep(0.5)
            continue
        if joystick_msg.buttons[0] == 1: # X pressed
            linear = 0.1
        elif joystick_msg.buttons[2] == 1: # Triangle pressed
            linear = -0.1
        if joystick_msg.axes[4] < 0:
            linear = 0.5 * -abs(joystick_msg.axes[4])
        elif joystick_msg.axes[4] > 0:
            linear = 0.5 * abs(joystick_msg.axes[4])
        if joystick_msg.axes[2] < 1.0: # L2 pressed
            angular = 1 * (-joystick_msg.axes[2] + 1)
        elif joystick_msg.axes[5] < 1.0: # R2 pressed
            angular = -1 * (-joystick_msg.axes[5] + 1)
        if joystick_msg.axes[3] < 0:
            angular = 0.5 * -abs(joystick_msg.axes[3])
        elif joystick_msg.axes[3] > 0:
            angular = 0.5 * abs(joystick_msg.axes[3])
        turtle.update_location(linear,0,0,0,0,angular)
        turtle.publish_location()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException: pass
