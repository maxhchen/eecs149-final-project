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
    raw_data = sock.recv(4096)

    # decode received data into interpretable data type (string)
    decoded_data = raw_data.decode()
    
    # strip string and split by specified delimiter; ignore the first and last elements to prevent
    # edge behavior with sending data over sockets.
    #
    # Example: "3.52" may get sent as "3." and "52." Because we use timer.sleep(0.25), our computer
    # receives slower than the ESP32 sends, resulting in a large "packet" containing multiple sensor readings.
    # In our testing with this method, the "3." would always be the final value in a packet, and the "52" would
    # always be the first value in a packet, meaning we can deal with this by ignoring those values.
    # Due to the speed at which we are sending sensor data, missing a single data point is not significant,
    # and we need to perform processing regardless, meaning the precision of the raw data does not matter.
    decoded_data = np.array(decoded_data.strip().split(delimiter))[1:-1] 

    # Remove any remaining null data
    decoded_data = decoded_data[decoded_data != '']

    # Parse characters as floating-point values
    data = decoded_data.astype(float)

    # As a sanity check, verify that the data array is not empty -- then, perform processing:
    if (len(data) != 0):
        '''
        Name: moving_average
        Description: Quick implementation of a moving average filter
        [float] = fn([float] x, int w)

        sequence:   data array = [float] (1, N)
        width:      filter width = int (1, 1)
        --
        rv:         array of averages = [float] (1, N - w + 1)
        '''
        def moving_average(sequence, width):
            return np.convolve(sequence, np.ones(width), 'valid') / width

        print(moving_average(data, len(data)))



    time.sleep(0.25)
### END OF COMMUNICATION LOOP ###

# sock.close()