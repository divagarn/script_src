import pygatt
import struct
import rospy
import time
import threading
import subprocess
import datetime

rospy.init_node("data_publisher")
device_macs = ["AD:C3:5A:FA:42:79", "83:0D:B2:C9:53:9B", "21:A3:F6:9D:E6:CC"]
characteristic_uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
max_retries = 10

last_connection_status = [False for _ in range(len(device_macs))]
DosageValues = [[] for _ in range(len(device_macs))]
adapters = [pygatt.GATTToolBackend() for _ in range(len(device_macs))]

for adapter in adapters:
    adapter.start()
    time.sleep(2)

device_obj_list = [None for _ in range(len(device_macs))]
dosage_data = []
Stop_flag = False

if rospy.has_param("/final_dose_value"):
    print("cleared ......")
    rospy.delete_param("/final_dose_value")

# Initialize flags for each device
device_flags = [False] * len(device_macs)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file_name = f"uv_dose_log_{timestamp}.txt"


def write_log(message):
    log_file = open(log_file_name, "a")
    log_file.write(f"{message}\n")
    log_file.close()


def check_bluetooth_device_status(device_mac):
    try:
        output = subprocess.check_output(["bluetoothctl", "info", device_mac], stderr=subprocess.STDOUT, text=True)

        if "Connected: yes" in output:
            return "Connected"
        else:
            return "Disconnected"

    except subprocess.CalledProcessError as e:
        return f"Error: {e}"


def connect_with_retries(device_mac, adapter, index):
    global device_obj_list
    mr = 0
    while not rospy.is_shutdown() and mr < max_retries:
        try:
            device = adapter.connect(device_mac, timeout=10)
            device_obj_list[index] = device

            return True
        except Exception as err:
            print(f"Connection to {device_mac} failed. Retrying...", err)
            time.sleep(mr)
            mr += 1
    time.sleep(5)

    # print(f"Max connection retries reached for {device_mac}.")
    return False


def disconnect_with_retries(device_mac, adapter, index):
    global device_obj_list
    mr = 0

    adapter.disconnect(device_obj_list[index])
    print('Disconnected')

    time.sleep(0.5)

    # print(f"Max connection retries reached for {device_mac}.")
    return False


def check_and_reconnect(device_mac, index_):
    global last_connection_status, device_flags, device_obj_list
    while not rospy.is_shutdown() and (not Stop_flag):
        status = check_bluetooth_device_status(device_mac)
        # print(f"Device {device_mac} status: {status}")
        if (status == "Disconnected"):
            device_flags[index_] = False
        if not device_flags[index_]:
            if (status == 'Connected'):
                disconnect_with_retries(device_mac, adapters[index_], index_)
            # device_flags[index_] = False
            ret_status = connect_with_retries(device_mac, adapters[index_], index_)
            if ret_status:
                device_flags[index_] = True
                print(f"Device {device_mac} connected successfully.")
            else:
                device_flags[index_] = False
                print(f"Device {device_mac} failed to reconnect.")

        if device_flags[index_] and (not last_connection_status[index_]):
            # print(f"Subscribing to characteristic for {device_mac}")
            device_obj_list[index_].subscribe(characteristic_uuid,callback=lambda handle, value: handle_data(handle, value, device_mac, index_))

            print('Calling....', device_mac)
        last_connection_status[index_] = device_flags[index_]
        # print(device_mac,device_flags[index_],last_connection_status[index_])
        time.sleep(5)


def handle_data(handle, value, device_mac, inidx):
    global received_data, DosageValues
    decoded_value = value.hex()
    value_length = len(value)

    if value_length == 11:
        received_data = struct.unpack("H", struct.pack("BB", value[7], value[6]))[0]
        # print(f"Received data from {device_mac}: {received_data} mJ/cm2")
        DosageValues[inidx].append(received_data)


threads = []
for i, device_mac in enumerate(device_macs):
    thread = threading.Thread(target=check_and_reconnect, args=(device_mac, i))
    time.sleep(1)
    # thread.daemon = True
    thread.start()
    # threads.append(thread)

while not rospy.is_shutdown():
    try:
        time.sleep(10)
        final_result_string = ""

        for di in range(len(device_macs)):
            if len(DosageValues[di]) > 0:
                final_result = f"{device_macs[di]};{device_flags[di]};{DosageValues[di][-1]}"
                write_log(f"{device_macs[di]};{device_flags[di]};{DosageValues[di][-1]}")
                final_result_string += final_result

                if di < len(device_macs) - 1:
                    final_result_string += ", "

        # Print and store the concatenated result as a ROS parameter
        print(final_result_string)
        rospy.set_param("/final_dose_value", final_result_string)

        time.sleep(10)
    except rospy.ROSInterruptException:
        for adapter in adapters:
            adapter.stop()
        break

Stop_flag = True
print('Calling the adapter close')
for i, device_mac in enumerate(device_macs):
    disconnect_with_retries(device_mac, adapters[i], i)
for adapter in adapters:
    adapter.stop()
