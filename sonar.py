#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, Range
import math

SONAR_FRONT_LEFT = "sonar_front_left"
SONAR_FRONT_RIGHT = "sonar_front_right"

SONAR_FRONT_LEFT_MEAN_DEGREE = 45
SONAR_FRONT_RIGHT_MEAN_DEGREE = 315

DIAGONAL_OFFSET = 0.35
SONAR_MAX_RANGE = 0.55
SONAR_DISTANCE_RATIO = 0.5778
MAX_SONAR_FIELD_DEGREE = 20
SONAR_ANGLE_REDUCTION = 10

FILTER_SIZE = 5
FILTER_THRESHOLD = 0.5


class LaserTest:
    def __init__(self):
        rospy.init_node('laser_scan')
        self.rate = rospy.Rate(10.0)
        self.status_pub = rospy.Publisher('/scan_from_range', LaserScan, queue_size=10)

        self.sonar_front_left_data = [float('inf')] * FILTER_SIZE
        self.sonar_front_right_data = [float('inf')] * FILTER_SIZE

        self.laser = LaserScan()
        self.laser.header.frame_id = "base_footprint"
        self.laser.header.stamp = rospy.Time.now()
        self.laser.scan_time = 0.0
        self.laser.time_increment = 0.0
        self.laser.angle_max = 6.28319
        self.laser.angle_min = 0.0
        self.laser.range_min = 0.0
        self.laser.range_max = 3.50
        self.laser.angle_increment = 0.0174533
        self.laser.ranges = [float('inf')] * 360

        rospy.Subscriber(SONAR_FRONT_LEFT, Range, self.sonar_front_left)
        rospy.Subscriber(SONAR_FRONT_RIGHT, Range, self.sonar_front_right)

    def apply_filter(self, data, data_list):
        data_list.append(data)
        if len(data_list) > FILTER_SIZE:
            data_list.pop(0)
        return data_list

    def publish_filtered_data(self, data_list, angle_mean):
        if len(data_list) == FILTER_SIZE and all(val < FILTER_THRESHOLD for val in data_list):
            self.laser.ranges[angle_mean] = data_list[-1]
            return True
        return False

    def sonar_front_left(self, data):
        mean_angle = SONAR_FRONT_LEFT_MEAN_DEGREE
        range_value = data.range
        self.sonar_front_left_data = self.apply_filter(range_value, self.sonar_front_left_data)
        if self.publish_filtered_data(self.sonar_front_left_data, mean_angle):
            self.status_pub.publish(self.laser)

    def sonar_front_right(self, data):
        mean_angle = SONAR_FRONT_RIGHT_MEAN_DEGREE
        range_value = data.range
        self.sonar_front_right_data = self.apply_filter(range_value, self.sonar_front_right_data)
        if self.publish_filtered_data(self.sonar_front_right_data, mean_angle):
            self.status_pub.publish(self.laser)


if __name__ == "__main__":
    laserObj = LaserTest()
    pre_scan = [float('inf')] * 360
    try:
        while not rospy.is_shutdown():
            laserObj.laser.header.stamp = rospy.Time.now()
            if pre_scan != laserObj.laser.ranges:
                laserObj.status_pub.publish(laserObj.laser)
                pre_scan = laserObj.laser.ranges
            laserObj.rate.sleep()
    except rospy.ROSInterruptException:
        pass
