#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys
import numpy as np
import signal
from sensor_msgs.msg import LaserScan

# Don't change
MAX_DEGREE = 45
ROBOT_RADIUS = 0.30
SCAN_TOPIC = "/scan_filtered"

# Change according to user input
LIDAR_OBSTACLE_SIDE_DETECTION_DISTANCE = 0.50
LIDAR_OBSTACLE_DETECTION_FORWARD_BACKWARD_DISTANCE = 0.60


class InterruptManager:
    RUNNING = True

    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.front_obstacle = False
        self.back_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.velocity = Twist()
        rospy.Subscriber(SCAN_TOPIC, LaserScan, self.scan_callback)

    def signal_handler(self, *args):
        print("exiting", args)
        self.RUNNING = False

    def scan_callback(self, data):
        scan_data = np.array(data.ranges)

        scan_data[np.isinf(scan_data)] = 0
        scan_data = np.round(scan_data, decimals=2)

        back = np.append(scan_data[(720 - int(MAX_DEGREE / 2) * 2):], scan_data[:(0 + int(MAX_DEGREE / 2) * 2)])
        left = scan_data[180 - int(MAX_DEGREE / 2) * 2:180 + int(MAX_DEGREE / 2) * 2]
        front = scan_data[360 - int(MAX_DEGREE / 2) * 2:360 + int(MAX_DEGREE / 2) * 2]
        right = scan_data[540 - int(MAX_DEGREE / 2) * 2:540 + int(MAX_DEGREE / 2) * 2]
        back_value = np.argwhere(np.logical_and(back <= LIDAR_OBSTACLE_DETECTION_FORWARD_BACKWARD_DISTANCE, back >= ROBOT_RADIUS))
        if back_value.size > 0:
            if len(back_value[0]):
                self.back_obstacle = True
                self.velocity.linear.x = 0
                velocity_pub.publish(self.velocity)
        else:
            self.back_obstacle = False

        left_value = np.argwhere(np.logical_and(left <= LIDAR_OBSTACLE_SIDE_DETECTION_DISTANCE, left >= ROBOT_RADIUS))
        if left_value.size > 0:
            if len(left_value[0]):
                self.left_obstacle = True
                self.velocity.angular.z = 0
                velocity_pub.publish(self.velocity)
        else:
            self.left_obstacle = False

        right_value = np.argwhere(np.logical_and(right <= LIDAR_OBSTACLE_SIDE_DETECTION_DISTANCE, right >= ROBOT_RADIUS))
        if right_value.size > 0:
            if len(right_value[0]):
                self.right_obstacle = True
                self.velocity.angular.z = 0
                velocity_pub.publish(self.velocity)
        else:
            self.right_obstacle = False

        front_value = np.argwhere(np.logical_and(front <= LIDAR_OBSTACLE_DETECTION_FORWARD_BACKWARD_DISTANCE,
                                                 front >= ROBOT_RADIUS))
        if front_value.size > 0:
            if len(front_value[0]):
                self.front_obstacle = True
                self.velocity.linear.x = 0
                velocity_pub.publish(self.velocity)
        else:
            self.front_obstacle = False


if __name__ == '__main__':
    rospy.init_node('joystick')
    rate = rospy.Rate(10.0)  # 10Hz
    velocity_pub = rospy.Publisher('/robot_smooth_cmd_vel', Twist, queue_size=10)
    interruptObj = InterruptManager()
    #interruptObj.velocity = Twist()
    print("Entered Joystick ...")
    try:
        rospy.set_param("/haystack/joystick/linear_velocity", "0.10")
        rospy.set_param("/haystack/joystick/angular_velocity", "0.17")
        rospy.set_param("/haystack/joystick/current_linear_velocity", "0.0")
        rospy.set_param("/haystack/joystick/current_angular_velocity", "0.0")
        rospy.set_param("/haystack/joystick/state", "RELEASED")
    except Exception as e:
        print("Ros Error :", e)
    while interruptObj.RUNNING:
        try:
            state = rospy.get_param("/haystack/joystick/state")
            mode = rospy.get_param("/haystack/mode")
        except:
            state = "RELEASED"
            mode = "IDLE"
        while state == "PRESSED" and (mode == "MANUAL" or mode == "DISINFECT"):
            interruptObj.velocity.linear.x = float(rospy.get_param("/haystack/joystick/current_linear_velocity"))
            interruptObj.velocity.angular.z = float(rospy.get_param("/haystack/joystick/current_angular_velocity"))

            # Check obstacle flags and decide whether to publish velocity
            if (interruptObj.velocity.linear.x != 0.0 and interruptObj.velocity.angular.z == 0.0) or (
                    interruptObj.velocity.linear.x == 0.0 and interruptObj.velocity.angular.z != 0.0):
                if interruptObj.velocity.linear.x > 0 and not interruptObj.front_obstacle:
                    velocity_pub.publish(interruptObj.velocity)

                if interruptObj.velocity.linear.x < 0 and not interruptObj.back_obstacle:
                    velocity_pub.publish(interruptObj.velocity)

                if interruptObj.velocity.angular.z > 0 and not interruptObj.left_obstacle:
                    velocity_pub.publish(interruptObj.velocity)

                if interruptObj.velocity.angular.z < 0 and not interruptObj.right_obstacle:
                    velocity_pub.publish(interruptObj.velocity)

            rate.sleep()
            state = rospy.get_param("/haystack/joystick/state")
        rate.sleep()
    print("Exiting Joystick...")
