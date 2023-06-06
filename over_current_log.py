#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from ubiquity_motor.msg import MotorState
import datetime
import signal

#CURRENT_THRESHOLD = 0.65
OVER_CURRENT_OCCURRENCE = 0
OVER_CURRENT_OCCURRENCE_LIMIT = 2

Sensor_Status = {"motor": True}

CURRENT_THRESHOLD = rospy.get_param("/haystack/motor_current_threshold",0.65)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file_name = f"robot_over_current_log_{timestamp}.txt"

log_file = open(log_file_name, 'a') 
leftCurrent = 0.0
rightCurrent = 0.0
run_process = True

def handler_stop_signals(signum, frame):
    global run_process
    run_process = False

def write_log(message):
    log_file.write(f"{message}\n") 	


def motor_state_callback(data):
    global leftCurrent,rightCurrent
    leftCurrent = data.leftCurrent
    rightCurrent = data.rightCurrent
        
def main_call():
	global OVER_CURRENT_OCCURRENCE
	global leftCurrent,rightCurrent
	signal.signal(signal.SIGINT, handler_stop_signals)
	signal.signal(signal.SIGTERM, handler_stop_signals)
	global run_process

	while(run_process):
		live_message = f"live_current. LeftCurrent: {leftCurrent}, RightCurrent: {rightCurrent}"
		write_log(live_message)
		print(live_message)

		if leftCurrent > CURRENT_THRESHOLD or rightCurrent > CURRENT_THRESHOLD:
			if OVER_CURRENT_OCCURRENCE > OVER_CURRENT_OCCURRENCE_LIMIT:
				try:
					mode = rospy.get_param("/haystack/mode")
				except rospy.ROSException:
					mode = "IDLE"

				if mode != "FOLLOW":
					message = f"Over Current Detected. LeftCurrent: {leftCurrent}, RightCurrent: {rightCurrent}"
					print(message)
					write_log(message)
					sys.exit()
				OVER_CURRENT_OCCURRENCE = 0

			OVER_CURRENT_OCCURRENCE += 1
		else:
			OVER_CURRENT_OCCURRENCE = 0


if __name__ == '__main__':
    print("Inside Robot Safety Node")
    rospy.init_node('Robot_Safety_Monitor', anonymous=True)
    rospy.Subscriber("motor_state", MotorState, motor_state_callback)
    main_call()
    log_file.close()
