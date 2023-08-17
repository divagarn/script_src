#!/usr/bin/env python3
# Maintainer: DIVAGAR N  n.divagar@mobiveil.co.in
# Copyright (c) mobiveil

import math
import rospy, sys
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from ubiquity_motor.msg import MotorState
from tf.transformations import euler_from_quaternion
import signal
import configparser
import datetime

X_AXIS = -110
Y_AXIS = -1

CURRENT_THRESHOLD = 0.75
SENSOR_MSG_INITIAL_TIMEOUT = 20  # sec
SENSOR_MSG_TIMEOUT = 2

OVER_CURRENT_OCCURRENCE = 0
OVER_CURRENT_OCCURRENCE_LIMIT = 10

SENSOR_FAILURE_OCCURRENCE = 0
SENSOR_FAILURE_OCCURRENCE_LIMIT = 2
PUBLISH_VELOCITY = True
sensor_data_timer = None
IMU_COUNTER = 0
DELAY_IMU_RATE = 20

Sensor_Status = {"motor": True, "lidar": True, "camera": True}
System_Status = {"battery": True, "arduino": True}

config = configparser.ConfigParser()
config.read("/haystack_disinfect_report/robot_config.ini")

if config.has_option("ROBOT", "MOTOR_CURRENT_THRESHOLD"):
    CURRENT_THRESHOLD = float(config["ROBOT"]["MOTOR_CURRENT_THRESHOLD"])

rospy.set_param("/haystack/motor_current_threshold", 0.65)

if config.has_option("ROBOT", "FRONT_TILT_TOLERANCE"):
    front_tilt_value = float(config["ROBOT"]["FRONT_TILT_TOLERANCE"])
else:
    front_tilt_value = 10
if config.has_option("ROBOT", "BACK_TILT_TOLERANCE"):
    back_tilt_value = float(config["ROBOT"]["BACK_TILT_TOLERANCE"])
else:
    back_tilt_value = 10

if config.has_option("ROBOT", "RIGHT_TILT_TOLERANCE"):
    right_tilt_value = float(config["ROBOT"]["RIGHT_TILT_TOLERANCE"])
else:
    right_tilt_value = 10

if config.has_option("ROBOT", "LEFT_TILT_TOLERANCE"):
    left_tilt_value = float(config["ROBOT"]["LEFT_TILT_TOLERANCE"])
else:
    left_tilt_value = 10

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file_name = f"robot_log_{timestamp}.txt"
# log_file = open(log_file_name, "w")

x_axis_upper_threshold = X_AXIS + front_tilt_value
x_axis_lower_threshold = X_AXIS - back_tilt_value

y_axis_upper_threshold = Y_AXIS + right_tilt_value
y_axis_lower_threshold = Y_AXIS - left_tilt_value


def write_log(message):
    log_file = open(log_file_name, "a")
    log_file.write(f"{message}\n")
    log_file.close()


def imu_callback(data):
    global OVER_CURRENT_OCCURRENCE, PUBLISH_VELOCITY, IMU_COUNTER
    value = [round(math.degrees(i)) for i in euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])]
    write_log(f"IMU LIVE VALUE: {value[0], value[1]}")

    if IMU_COUNTER > DELAY_IMU_RATE:
        if x_axis_lower_threshold > value[0] or value[0] > x_axis_upper_threshold or y_axis_lower_threshold > value[1] or value[1] > y_axis_upper_threshold:
            try:
                mode = rospy.get_param("/haystack/mode")
            except:
                mode = "IDLE"
            if mode != "FOLLOW":
                log_message = f"IMU DETECTED: Robot got tilted! Orientation: {value[0], value[1]}"
                write_log(log_message)
                OVER_CURRENT_OCCURRENCE = 0
                rospy.signal_shutdown("IMU")

        IMU_COUNTER = 0
    else:
        IMU_COUNTER += 1


def motor_state_callback(data):
    global OVER_CURRENT_OCCURRENCE, PUBLISH_VELOCITY
    Sensor_Status["motor"] = True
    write_log(f"Left_Current: {data.leftCurrent}, Right_Current: {data.rightCurrent}")
    if data.leftCurrent > CURRENT_THRESHOLD or data.rightCurrent > CURRENT_THRESHOLD:
        if OVER_CURRENT_OCCURRENCE > OVER_CURRENT_OCCURRENCE_LIMIT:
            try:
                mode = rospy.get_param("/haystack/mode")
            except:
                mode = "IDLE"
            if mode != "FOLLOW":
                log_message = f"MOTOR CURRENT DETECTED: Over Current Detected, Bumping Detected. Left Current: {data.leftCurrent}, Right Current: {data.rightCurrent}"
                write_log(log_message)
                OVER_CURRENT_OCCURRENCE = 0
                # log_file.close()
                rospy.signal_shutdown("over current")

        OVER_CURRENT_OCCURRENCE += 1
    else:
        OVER_CURRENT_OCCURRENCE = 0


if __name__ == '__main__':
    print("Inside Robot Safety Node")
    rospy.init_node('Robot_Safety_Monitor', anonymous=True)
    robot_safety_publisher = rospy.Publisher('robot_safety', String, queue_size=10)
    rospy.Subscriber("imu/data", Imu, imu_callback)
    rospy.Subscriber("motor_state", MotorState, motor_state_callback)
    rospy.spin()
