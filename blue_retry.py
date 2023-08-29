import pygatt
import struct
import rospy
import time

rospy.init_node("data_publisher")
device_mac = "83:0D:B2:C9:53:9B"
characteristic_uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
max_retries = 3  

adapter = pygatt.GATTToolBackend()


def connect_with_retries(max_retries):
    for _ in range(max_retries):
        try:
            adapter.start()
            device = adapter.connect(device_mac)
            return device
        except pygatt.exceptions.NotConnectedError:
            print("Connection failed. Retrying...")
            time.sleep(2)
    print("Max connection retries reached.")
    return None

try:
    device = connect_with_retries(max_retries)
    if device is None:
        raise Exception("Unable to establish a connection.")

    received_data = None

    def handle_data(handle, value):
        global received_data
        decoded_value = value.hex()
        value_length = len(value)
        print("Received:", decoded_value)
        print("Received Length:", value_length)

        if value_length == 11:
            received_data = struct.unpack("H", struct.pack("BB", value[7], value[6]))[0]
            print(received_data, "mJ/cm2")

    device.subscribe(characteristic_uuid, callback=handle_data)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if received_data is not None:
            rospy.set_param("received_data", received_data)
            received_data = None

        rate.sleep()

except KeyboardInterrupt:
    pass
