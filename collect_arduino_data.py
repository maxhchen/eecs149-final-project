import bluetooth 
import time
import numpy as np

# Board Specifics: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/overview
# Pintout for the board : https://makeabilitylab.github.io/physcomp/esp32/esp32.html
# https://people.csail.mit.edu/albert/bluez-intro/x290.html

# ESP32 Metadata
bd_addr = "24:62:AB:D2:A7:06"
port = 1

# Establish BLE Socket to communicate between computer and ESP32 board
sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_addr, port))


delimiter = '|'

# Communication loop
while True: 
    # # Send data from computer to ESP32:
    # data = "hello!\n"
    # sock.send(data.encode())

    # param = buffer width, # Bytes to receive over socket
    receive_data = sock.recv(4096)

    # decode received data into interpretable data type (string)
    decoded_data = receive_data.decode()
    
    # strip string and split by specified delimiter; ignore the first and last elements to prevent
    # edge behavior with sending data over sockets.
    #
    # Example: "3.52" may get sent as "3." and "52." Because we use timer.sleep(0.25), our computer
    # receives slower than the ESP32 sends, resulting in a large "packet" containing multiple sensor readings.
    # In our testing with this method, the "3." would always be the final value in a packet, and the "52" would
    # always be the first value in a packet, meaning we can deal with this by ignoring those values.
    # Due to the speed at which we are sending sensor data, missing a single data point is not significant,
    # and we need to perform processing regardless, meaning the precision of the raw data does not matter.
    split_data = np.array(decoded_data.strip().split(delimiter))[1:-1] 

    # Remove any remaining null data
    split_data = split_data[split_data != '']

    # Parse characters as floating-point values
    float_data = split_data.astype(float)

    # As a sanity check, verify that the data array is not empty -- then, perform processing:
    if (len(float_data) != 0):
        print(np.round(np.mean(float_data), 2))
    time.sleep(0.25)
 
    # print(receive_data.decode())
    # print(ord(receive_data.decode()))

# sock.close()