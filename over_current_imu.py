#!/usr/bin/env python3
# Maintainer: DIVAGAR N  n.divagar@mobiveil.co.in
# Copyright (c) mobiveil

import math
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Imu, LaserScan, Image, Range
from ubiquity_motor.msg import MotorState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import signal
import threading
import datetime

X_AXIS = -110
Y_AXIS = -1

CURRENT_THRESHOLD = 0.75
SENSOR_MSG_INITIAL_TIMEOUT = 20
SENSOR_MSG_TIMEOUT = 2

OVER_CURRENT_OCCURRENCE = 0
OVER_CURRENT_OCCURRENCE_LIMIT = 10

SENSOR_FAILURE_OCCURRENCE = 0
SENSOR_FAILURE_OCCURRENCE_LIMIT = 2
PUBLISH_VELOCITY = True
sensor_data_timer = None
IMU_OCCURRENCE = 0
VALUE = None
IMU_OCCURRENCE_LIMIT = 2

Sensor_Status = {"motor": True, "lidar": True, "camera": True}
System_Status = {"battery": True, "arduino": True}

rospy.set_param("/haystack/motor_current_threshold", 0.75)

front_tilt_value = 3

back_tilt_value = 3

right_tilt_value = 5

left_tilt_value = 5

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file_name = f"robot_log_{timestamp}.txt"

x_axis_upper_threshold = X_AXIS + front_tilt_value
x_axis_lower_threshold = X_AXIS - back_tilt_value

y_axis_upper_threshold = Y_AXIS + right_tilt_value
y_axis_lower_threshold = Y_AXIS - left_tilt_value


def write_log(message):
    log_file = open(log_file_name, "a")
    log_file.write(f"{message}\n")
    log_file.close()


def imu_callback(data):
    global VALUE
    VALUE = [round(math.degrees(i)) for i in euler_from_quaternion([data.orientation.x, data.orientation.y,
                                                                    data.orientation.z, data.orientation.w])]
    write_log(f"IMU DATA: {VALUE[0], VALUE[1]}")
    print(f"IMU DATA: {VALUE[0], VALUE[1]}")


def motor_state_callback(data):
    global OVER_CURRENT_OCCURRENCE, PUBLISH_VELOCITY, VALUE, IMU_OCCURRENCE
    Sensor_Status["motor"] = True
    write_log(f"MOTOR CURRENT : Left_Current: {data.leftCurrent}, Right_Current: {data.rightCurrent}")
    print(f"MOTOR CURRENT : Left_Current: {data.leftCurrent}, Right_Current: {data.rightCurrent}")
    if data.leftCurrent > CURRENT_THRESHOLD or data.rightCurrent > CURRENT_THRESHOLD:
        try:
            mode = rospy.get_param("/haystack/mode")
        except:
            mode = "IDLE"

        if OVER_CURRENT_OCCURRENCE > OVER_CURRENT_OCCURRENCE_LIMIT:

            if mode != "FOLLOW":
                print("Over Current Detected")
                log_message = f">>>>>>>>>>>MOTOR CURRENT DETECTED : Left Current: {data.leftCurrent}, Right Current: {data.rightCurrent}"
                write_log(log_message)
                print(f"MOTOR CURRENT DETECTED : Left Current: {data.leftCurrent}, Right Current: {data.rightCurrent}")
                OVER_CURRENT_OCCURRENCE = 0
                rospy.signal_shutdown("over current")

        else:
            OVER_CURRENT_OCCURRENCE += 1
            if x_axis_lower_threshold > VALUE[0] or VALUE[0] > x_axis_upper_threshold or \
                    y_axis_lower_threshold > VALUE[1] or VALUE[1] > y_axis_upper_threshold:
                if IMU_OCCURRENCE > IMU_OCCURRENCE_LIMIT:

                    print("ROBOT GOT TILTED !")
                    log_message = f">>>>>>>>>>>>>>>ROBOT GOT TILTED : DATA : {VALUE[0], VALUE[1]}"
                    write_log(log_message)
                    print(f"ROBOT GOT TILTED : DATA: {VALUE[0], VALUE[1]}")
                    IMU_OCCURRENCE = 0
                    OVER_CURRENT_OCCURRENCE = 0
                    rospy.signal_shutdown("IMU")
                else:
                    IMU_OCCURRENCE += 1

    else:
        OVER_CURRENT_OCCURRENCE = 0
        PUBLISH_VELOCITY = True


if __name__ == '__main__':
    print("Inside Robot Safety Node")
    rospy.init_node('Robot_Safety_Monitor', anonymous=True)
    robot_safety_publisher = rospy.Publisher('robot_safety', String, queue_size=10)
    rospy.Subscriber("imu/data", Imu, imu_callback)
    rospy.Subscriber("motor_state", MotorState, motor_state_callback)
    rospy.spin()
