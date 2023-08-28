import pygatt
import struct
import rospy

rospy.init_node("data_modifier")

device_mac = "83:0D:B2:C9:53:9B"
characteristic_uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

adapter = pygatt.GATTToolBackend()

try:
    adapter.start()
    device = adapter.connect(device_mac)
    received_data = None


    def handle_data(handle, value):
        global received_data
        value_length = len(value)

        if value_length == 11:
            received_data = bytearray(value)
            received_data[6] = 0
            received_data[7] = 0
            print("Modified Data:", received_data.hex())

            # Write the modified data back to the device
            device.char_write(characteristic_uuid, received_data, wait_for_response=True)
            print("Data written to the device")


    device.subscribe(characteristic_uuid, callback=handle_data)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

except KeyboardInterrupt:
    pass
