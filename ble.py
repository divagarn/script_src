import pygatt
import struct
device_mac = "83:0D:B2:C9:53:9B"
characteristic_uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

adapter = pygatt.GATTToolBackend()

try:
    adapter.start()
    device = adapter.connect(device_mac)

    def handle_data(handle, value):
        print(value)
        decoded_value = value.hex()
        value_length = len(value)
        print("Received:", decoded_value)
        print("Received Length:", value_length)

        if value_length == 11:
            # seventh_byte = 
            # eighth_byte = value[7]
            # print(seventh_byte, eighth_byte)
            # print("7th Byte:", seventh_byte)
            # print("8th Byte:", eighth_byte0)
            print(struct.unpack("H", struct.pack("BB",value[7], value[6]))[0])
            dose_val = struct.unpack("H", struct.pack("BB",value[7], value[6]))[0]
            print("DOSE VALUE : " , dose_val)

    device.subscribe(characteristic_uuid, callback=handle_data)

    input("Press Enter to stop monitoring...")
finally:
    adapter.stop()
