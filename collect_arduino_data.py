status = 1 # 0 = default, 1 = debug

if (status == 0):
    import bluetooth 
import time
import numpy as np

from generate_synthetic_data import generate_synthetic_data

# Board Specifics: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/overview
# Pintout for the board : https://makeabilitylab.github.io/physcomp/esp32/esp32.html
# https://people.csail.mit.edu/albert/bluez-intro/x290.html

# ESP32 Metadata
bd_addr = "24:62:AB:D2:A7:06"
port = 1

THUMB = 1
INDEX = 2
MIDDLE = 3
RING = 4

saturate_values = np.array([200, 1500, 3000, 500])

def receive_data_over_bluetooth():
    # param = buffer width, # Bytes to receive over socket
    if (status != 0):
        return ValueError("In debug mode, yet called receive_data_over_bluetooth()")
    raw_data = sock.recv(4096)

    # decode received data into interpretable data type (string)
    decoded_data = raw_data.decode()
    return decoded_data

if __name__ == '__main__':
    # Establish BLE Socket to communicate between computer and ESP32 board
    if (status == 0):
        sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((bd_addr, port))

    packet_delimiter = '|'
    value_delimiter = '_'

    # Communication loop
    while True: 
        # # Send data from computer to ESP32:
        # data = "hello!\n"
        # sock.send(data.encode())

        if status == 0:
            decoded_data = receive_data_over_bluetooth()
            # strip string and split by specified delimiter; ignore the first and last elements to prevent
            # edge behavior with sending data over sockets.
            #
            # Example: "3.52" may get sent as "3." and "52." Because we use timer.sleep(0.25), our computer
            # receives slower than the ESP32 sends, resulting in a large "packet" containing multiple sensor readings.
            # In our testing with this method, the "3." would always be the final value in a packet, and the "52" would
            # always be the first value in a packet, meaning we can deal with this by ignoring those values.
            # Due to the speed at which we are sending sensor data, missing a single data point is not significant,
            # and we need to perform processing regardless, meaning the precision of the raw data does not matter.
            decoded_packet = np.array(decoded_data.strip().split(packet_delimiter))[1:-1]

            # Remove any remaining null data
            decoded_packet = decoded_packet[decoded_data != '']

            # Break apart packets
            data = decoded_data.char.split(value_delimiter)
            # Parse characters as floating-point values
            data = data.astype(float)

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

                print("THUMB: " + moving_average(data[THUMB, :], len(data)))
                print("INDEX: " + moving_average(data[INDEX, :], len(data)))
                print("MIDDLE: " + moving_average(data[MIDDLE, :], len(data)))
                print("RING: " + moving_average(data[RING, :], len(data)))
        else:
            decoded_data = generate_synthetic_data()
            # print(decoded_data)

            decoded_packet = np.array(decoded_data.strip().split(packet_delimiter))
            # print(decoded_packet)
            # print(len(decoded_packet))

            # Remove any remaining null data
            # decoded_packet = decoded_packet[decoded_data != '']
            # print(decoded_packet)
            # print(len(decoded_packet))

            # Break apart packets
            data = np.array([x.split(value_delimiter) for x in decoded_packet])[:, 1:-1]
            # print(data)
            data = data[data != ''].reshape(data.shape)
            # print(data)
            data = np.round(data.astype(np.float)).astype(np.int)
            # print(data)

            [thumb, index, middle, ring] = data
            avg = np.average(data, 1)

            print(avg > saturate_values / 2)

        time.sleep(0.25)
    ### END OF COMMUNICATION LOOP ###

    # sock.close()


