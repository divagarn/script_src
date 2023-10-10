import pygatt
import struct
import rospy
import time
import threading
import subprocess

rospy.init_node("data_publisher")
device_macs = ["AD:C3:5A:FA:42:79", "83:0D:B2:C9:53:9B", "21:A3:F6:9D:E6:CC"]
characteristic_uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
max_retries = 10

adapters = [pygatt.GATTToolBackend() for _ in range(len(device_macs))]

for adapter in adapters:
    adapter.start()
    time.sleep(2)

device_obj_list = []
dosage_data = []

# Initialize flags for each device
device_flags = [False] * len(device_macs)


def check_bluetooth_device_status(device_mac):
    try:
        output = subprocess.check_output(["bluetoothctl", "info", device_mac], stderr=subprocess.STDOUT, text=True)

        if "Connected: yes" in output:
            return "Connected"
        elif "Paired: yes" in output:
            return "Paired but not connected"
        else:
            return "Disconnected"

    except subprocess.CalledProcessError as e:
        return f"Error: {e}"


def connect_with_retries(device_mac, adapter, index):
    mr = 0
    while not rospy.is_shutdown() and mr < max_retries:
        try:
            device = adapter.connect(device_mac)
            device_obj_list.append(device)
            print(f"{device_mac} CONNECTED")
            # Set the flag for the connected device to True
            device_flags[index] = True
            return True
        except:
            # print(f"Connection to {device_mac} failed. Retrying...")
            time.sleep(mr)
            mr += 1
    time.sleep(5)

    print(f"Max connection retries reached for {device_mac}.")
    return False


def check_and_reconnect(device_mac, index):
    while not rospy.is_shutdown():
        status = check_bluetooth_device_status(device_mac)
        print(f"Device {device_mac} status: {status}")

        if status == "Disconnected":
            ret_status = connect_with_retries(device_mac, adapters[index], index)
            if ret_status:
                print(f"Device {device_mac} reconnected successfully.")
            else:
                print(f"Device {device_mac} failed to reconnect.")

        elif status == "Connected":
            # Subscribe to the characteristic when connected
            # print(f"Subscribing to characteristic for {device_mac}")
            device_obj_list[0].subscribe(characteristic_uuid,
                                         callback=lambda handle, value: handle_data(handle, value, device_mac))
            device_obj_list[1].subscribe(characteristic_uuid,
                                         callback=lambda handle, value: handle_data2(handle, value, device_mac))
            device_obj_list[2].subscribe(characteristic_uuid,
                                         callback=lambda handle, value: handle_data3(handle, value, device_mac))

        time.sleep(3)


def handle_data(handle, value, device_mac):
    global received_data
    decoded_value = value.hex()
    value_length = len(value)

    if value_length == 11:
        received_data = struct.unpack("H", struct.pack("BB", value[7], value[6]))[0]
        print(f"Received data from {device_mac}: {received_data} mJ/cm2")


def handle_data2(handle, value, device_mac):
    global received_data
    decoded_value = value.hex()
    value_length = len(value)

    if value_length == 11:
        received_data = struct.unpack("H", struct.pack("BB", value[7], value[6]))[0]
        print(f"Received data from {device_mac}: {received_data} mJ/cm2")


def handle_data3(handle, value, device_mac):
    global received_data
    decoded_value = value.hex()
    value_length = len(value)

    if value_length == 11:
        received_data = struct.unpack("H", struct.pack("BB", value[7], value[6]))[0]
        print(f"Received data from {device_mac}: {received_data} mJ/cm2")


threads = []
for i, device_mac in enumerate(device_macs):
    thread = threading.Thread(target=check_and_reconnect, args=(device_mac, i))
    thread.daemon = True
    thread.start()
    threads.append(thread)

while not rospy.is_shutdown():
    try:
        time.sleep(1)
    except rospy.ROSInterruptException:
        for adapter in adapters:
            adapter.stop()
        break

print('Calling the adapter close')
for adapter in adapters:
    adapter.stop()
