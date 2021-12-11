status = 0 # 0 = default, 1 = debug

if (status == 0):
    import bluetooth 
import time
import numpy as np

from generate_synthetic_data import generate_synthetic_data
from drone import * 

# Board Specifics: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/overview
# Pintout for the board : https://makeabilitylab.github.io/physcomp/esp32/esp32.html
# https://people.csail.mit.edu/albert/bluez-intro/x290.html

# ESP32 Metadata
bd_addr = "24:62:AB:D2:A7:06"
# bd_addr = "d8:87:02:70:0b:13" # BLE nano board
port = 1

INDEX = 1
MIDDLE = 2
RING = 3
PINKY = 4

offset = 5

def moving_average(sequence, width):
    return np.convolve(sequence, np.ones(width), 'valid') / width

def receive_data_over_bluetooth():
    # param = buffer width, # Bytes to receive over socket
    if (status != 0):
        return ValueError("In debug mode, yet called receive_data_over_bluetooth()")
    raw_data = sock.recv(4096)

    # decode received data into interpretable data type (string)
    decoded_data = raw_data.decode()
    return decoded_data

def calibrate_imu(num_times, packet_delimiter, value_delimiter):
    accel_values = np.zeros(3)
    gyro_values = np.zeros(3)

    while(num_times > 0):
        decoded_data = receive_data_over_bluetooth()
        if decoded_data.find(packet_delimiter) != -1:
                decoded_packet = np.array(decoded_data.strip().split(packet_delimiter)) # [1:-1]
        else:
            decoded_packet = decoded_data
        if "0." in decoded_packet[-1] or "_" in decoded_packet[-1]:
            decoded_packet = decoded_packet[:-1]
        if len(decoded_packet[-1]) == 1:
            decoded_packet = decoded_packet[:-1]
        print(decoded_packet)
        print(decoded_packet != '')
        decoded_packet = decoded_packet[decoded_packet != '']
        data = np.array([i.split(value_delimiter) for i in decoded_packet])

        # Parse characters as floating-point values
        data = data.astype(float)
        accel_values += np.array([moving_average(data[:, 0], len(data))[0], moving_average(data[:, 1], len(data))[0], moving_average(data[:, 2], len(data))[0]])
        gyro_values += np.array([moving_average(data[:, 3], len(data))[0],  moving_average(data[:, 4], len(data))[0], moving_average(data[:, 5], len(data))[0]])
    
    # Take the average values
    accel_values_avg = accel_values / num_times
    gyro_values_avg = gyro_values / num_times
    return accel_values_avg, gyro_values_avg

saturate_values = np.array([200, 1500, 3000, 500])

'''
    Name: moving_average
    Description: Quick implementation of a moving average filter
    [float] = fn([float] x, int w)

    sequence:   data array = [float] (1, N)
    width:      filter width = int (1, 1)
    --
    rv:         array of averages = [float] (1, N - w + 1)
'''

if __name__ == '__main__':
    # Establish BLE Socket to communicate between computer and ESP32 board
    if (status == 0):

        sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        sock.connect((bd_addr, port))


    packet_delimiter = '|'
    value_delimiter = '_'

    accel_offset, gyro_offset = calibrate_imu(15, packet_delimiter, value_delimiter)
    print("Finished Calibrating IMU")

    print("Initializing Drone")
    drone = init_drone()

    # send_rc_control(self, left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)

    # Communication loop
    while True: 
        # # Send data from computer to ESP32:


        if status == 0:
            decoded_data = receive_data_over_bluetooth()
            # print("A:", decoded_data)
            
            # strip string and split by specified delimiter; ignore the first and last elements to prevent
            # edge behavior with sending data over sockets.
            #
            # Example: "3.52" may get sent as "3." and "52." Because we use timer.sleep(0.25), our computer
            # receives slower than the ESP32 sends, resulting in a large "packet" containing multiple sensor readings.
            # In our testing with this method, the "3." would always be the final value in a packet, and the "52" would
            # always be the first value in a packet, meaning we can deal with this by ignoring those values.
            # Due to the speed at which we are sending sensor data, missing a single data point is not significant,
            # and we need to perform processing regardless, meaning the precision of the raw data does not matter.
      
            if decoded_data.find(packet_delimiter) != -1:
                decoded_packet = np.array(decoded_data.strip().split(packet_delimiter)) # [1:-1]
            else:
                decoded_packet = decoded_data
            if "0." in decoded_packet[-1] or "_" in decoded_packet[-1]:
                decoded_packet = decoded_packet[:len(decoded_packet)-1]
                print("hello")

            print("B:", decoded_packet)
            if len(decoded_packet[-1]) == 1:
                decoded_packet = decoded_packet[:-1]
            decoded_packet = decoded_packet[decoded_packet != '']
            print("B.5:", decoded_packet)

            # Break apart packets
            data = np.array([i.split(value_delimiter) for i in decoded_packet])
            # print("C:", data)

            # Parse characters as floating-point values
            data = data.astype(float)
            # print("D:", data)

            # As a sanity check, verify that the data array is not empty -- then, perform processing:
            if (len(data) != 0):

                accel_values = accel_offset - np.array([moving_average(data[:, 0], len(data))[0], moving_average(data[:, 1], len(data))[0], moving_average(data[:, 2], len(data))[0]])
                gyro_values = gyro_offset - np.array([moving_average(data[:, 3], len(data))[0],  moving_average(data[:, 4], len(data))[0], moving_average(data[:, 5], len(data))[0]])
                send_rc_command(drone, accel_values, gyro_values)
                # print(data.shape)
                print("accel_x:\t",  moving_average(data[:, 0], len(data))[0])
                print("accel_y:\t",  moving_average(data[:, 1], len(data))[0])
                print("accel_z:\t",  moving_average(data[:, 2], len(data))[0])
                print("gyro_x:\t",  moving_average(data[:, 3], len(data))[0])
                print("gyro_y:\t",  moving_average(data[:, 4], len(data))[0])
                print("gyro_z:\t",  moving_average(data[:, 5], len(data))[0])
                print("INDEX:\t\t",  moving_average(data[:, offset + INDEX], len(data))[0])
                print("MIDDLE:\t\t", moving_average(data[:, offset + MIDDLE], len(data))[0])
                print("RING:\t\t",   moving_average(data[:, offset + RING], len(data))[0])
                print("PINKY:\t\t",  moving_average(data[:, offset + PINKY], len(data))[0])
                print("-- -- -- -- -- -- -- --")
            

            

        else:
            decoded_data = generate_synthetic_data()

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


