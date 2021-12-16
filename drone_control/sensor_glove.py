import bluetooth 
import time
import numpy as np
import threading

from utils import * 

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

def print_debug(data): 
    print("accel_x:\t",  np.round(moving_average(data[:, 0], len(data))[0], 2))
    print("accel_y:\t",  np.round(moving_average(data[:, 1], len(data))[0], 2))
    print("accel_z:\t",  np.round(moving_average(data[:, 2], len(data))[0], 2))
    print("gyro_x:\t\t",  np.round(moving_average(data[:, 3], len(data))[0], 2))
    print("gyro_y:\t\t",  np.round(moving_average(data[:, 4], len(data))[0], 2))
    print("gyro_z:\t\t",  np.round(moving_average(data[:, 5], len(data))[0], 2))
    print("INDEX:\t\t",  np.round(moving_average(data[:, offset + INDEX], len(data))[0], 2))
    print("MIDDLE:\t\t", np.round(moving_average(data[:, offset + MIDDLE], len(data))[0], 2))
    print("RING:\t\t",   np.round(moving_average(data[:, offset + RING], len(data))[0], 2))
    print("PINKY:\t\t",  np.round(moving_average(data[:, offset + PINKY], len(data))[0], 2))
    print("-- -- -- -- -- -- -- --")

class SensorGlove:

    INDEX = 1
    MIDDLE = 2
    RING = 3
    PINKY = 4

    packet_delimiter = '|'
    value_delimiter = '_'

    def __init__(self):
        self.stop_event = threading.Event()
        self.finger = None
        self.forward_backward = 0
        self.left_right = 0

        self.hold = False
        self.held_data = ''
        self.past_accel_values = None
        self.decoded_data = None

        self.accel_offset = 0
        self.gyro_offset = 0

        self.connect()

    def connect(self):
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.sock.connect((bd_addr, port))
        self.sock.settimeout(.5)
        print('Timeout:', self.sock.gettimeout())
        print("Arduino connected")

    def receive_data_over_bluetooth(self):
        # param = buffer width, # Bytes to receive over socket
        # deadline = time.time() + 20.0

        # print('hanging here')
        raw_data = self.sock.recv(1024)
        # print('recv done')
        # decode received data into interpretable data type (string)
        decoded_data = raw_data.decode()
        return decoded_data

    def calibrate_imu(self, num_times):
        accel_values = np.zeros(3)
        gyro_values = np.zeros(3)
        num_elem = num_times

        hold = False
        held_data = ''

        while(num_times > 0):
            if not hold:
                decoded_data = self.receive_data_over_bluetooth()
            else:
                decoded_data = held_data + self.receive_data_over_bluetooth()
                hold = False
            # print("A:", decoded_data)
            if (decoded_data == "0.") or (decoded_data == "-"):
                hold = True
                held_data = decoded_data
                continue
            if decoded_data[-1] != self.packet_delimiter:
                hold = True
                held_data = decoded_data
                continue

            if decoded_data.find(self.packet_delimiter) != -1:
                decoded_packet = np.array(decoded_data.strip().split(self.packet_delimiter)) # [1:-1]
            else:
                decoded_packet = decoded_data
            
            if "0." in decoded_packet[-1] or "_" in decoded_packet[-1]:
                decoded_packet = decoded_packet[:-1]
            
            if len(decoded_packet[-1]) == 1:
                decoded_packet = decoded_packet[:-1]
            
            #print("B:", decoded_packet)
            #print("C:", decoded_packet != '')
            decoded_packet = decoded_packet[decoded_packet != '']
            # print("D:", decoded_packet)
            data = np.array([i.split(self.value_delimiter) for i in decoded_packet])
            #print("E:", data)

            # Parse characters as floating-point values
            data = data.astype(float)
            print("F:", data)
            
            accel_values += np.array([moving_average(data[:, 0], len(data))[0], moving_average(data[:, 1], len(data))[0], moving_average(data[:, 2], len(data))[0]])
            gyro_values += np.array([moving_average(data[:, 3], len(data))[0],  moving_average(data[:, 4], len(data))[0], moving_average(data[:, 5], len(data))[0]])
            num_times = num_times - 1
            
            time.sleep(0.1)

        # Take the average values
        accel_values_avg = accel_values / num_elem
        gyro_values_avg = gyro_values / num_elem

        self.accel_offset, self.gyro_offset = accel_values_avg, gyro_values_avg

    def get_data(self):
        # self.hold = False
        # self.held_data = ''
        # if not self.hold:
        #     decoded_data = self.receive_data_over_bluetooth()
        # else:
        #     decoded_data = self.held_data + self.receive_data_over_bluetooth()
        #     self.hold = False
        decoded_data = self.decoded_data
        if decoded_data is None:
            self.left_right = 0
            self.forward_backward = 0
            return

        # print('read data')
        # print(decoded_data)
        if self.packet_delimiter not in decoded_data:
            self.left_right = 0
            self.forward_backward = 0
            return
        
        first_delim = decoded_data.index(self.packet_delimiter)
        last_delim = len(decoded_data) -1 -  decoded_data[::-1].index(self.packet_delimiter)
        decoded_packet = decoded_data[first_delim+1:last_delim]

        # print(decoded_packet)

        # if (decoded_data == "0.") or (decoded_data == "-"):
        #     self.hold = True
        #     self.held_data = decoded_data
        #     return
        # if decoded_data[-1] != self.packet_delimiter:
        #     self.hold = True
        #     self.held_data = decoded_data
        #     return
        # print("didn't return")
        if decoded_data.find(self.packet_delimiter) != -1:
            decoded_packet = np.array(decoded_data.strip().split(self.packet_delimiter)) # [1:-1]
        else:
            decoded_packet = decoded_data
        
        if "0." in decoded_packet[-1] or "_" in decoded_packet[-1]:
            decoded_packet = decoded_packet[:-1]
        
        if len(decoded_packet[-1]) == 1:
            decoded_packet = decoded_packet[:-1]
        
        decoded_packet = decoded_packet[decoded_packet != '']
        # print("D:", decoded_packet)

        data = []
        try:
            for p in decoded_packet:
                s = p.split(self.value_delimiter)
                if len(s) != 10:
                    continue
                data.append(s)
            # Parse characters as floating-point values
            data = np.array(data, dtype=float)
            # data = np.array([i.split(self.value_delimiter) for i in decoded_packet])
        except Exception as e:
            print(e)
            self.left_right = 0
            self.forward_backward = 0
            return
        
        self.data = data
        
        # strip string and split by specified delimiter; ignore the first and last elements to prevent
        # edge behavior with sending data over sockets.
        #
        # Example: "3.52" may get sent as "3." and "52." Because we use timer.sleep(0.25), our computer
        # receives slower than the ESP32 sends, resulting in a large "packet" containing multiple sensor readings.
        # In our testing with this method, the "3." would always be the final value in a packet, and the "52" would
        # always be the first value in a packet, meaning we can deal with this by ignoring those values.
        # Due to the speed at which we are sending sensor data, missing a single data point is not significant,
        # and we need to perform processing regardless, meaning the precision of the raw data does not matter.

        # As a sanity check, verify that the data array is not empty -- then, perform processing:
        if (len(data) != 0):
            accel_values = np.array([moving_average(data[:, 0], len(data))[0], moving_average(data[:, 1], len(data))[0], moving_average(data[:, 2], len(data))[0]]) - self.accel_offset
            gyro_values = np.array([moving_average(data[:, 3], len(data))[0],  moving_average(data[:, 4], len(data))[0], moving_average(data[:, 5], len(data))[0]]) - self.gyro_offset
            index_finger_value = np.round(moving_average(data[:, offset + INDEX], len(data))[0], 2)
            middle_finger_value =  np.round(moving_average(data[:, offset + MIDDLE], len(data))[0], 2)
            ring_finger_value = np.round(moving_average(data[:, offset + RING], len(data))[0], 2)
            pinky_finger_value = np.round(moving_average(data[:, offset + PINKY], len(data))[0], 2)

            if self.past_accel_values is not None:
                self.left_right, self.forward_backward = find_rc_command(self.past_accel_values, accel_values, True)
            
            self.finger = find_special_command(index_finger_value, middle_finger_value, ring_finger_value, pinky_finger_value)                

            # x_val, y_val, z_val = accel_values - (self.past_accel_values if self.past_accel_values is not None else 0)
            # self.x_tilt = np.arctan(x_val / np.sqrt(y_val*y_val + z_val*z_val)) * 180.0 / np.pi;
            # self.y_tilt = np.arctan(y_val / np.sqrt(x_val*x_val + z_val*z_val)) * 180.0 / np.pi;
            # self.z_tilt = np.arctan(np.sqrt(x_val*x_val + y_val*y_val) / z_val) * 180.0 / np.pi;
            # print("Tilt:", self.x_tilt, self.y_tilt, self.z_tilt)

            print(self.left_right, self.forward_backward, self.finger)

            if self.past_accel_values is None:
                self.past_accel_values = accel_values

    def process(self):
        print("Started Arduino processing")
        self.calibrate_imu(50)
        self.calibrated = True
        print("Finished Calibrating IMU")

        while True:
            self.get_data()
            # print('before sleep')
            start = time.time()
            while time.time() - start < .07:
                pass
            # print('after sleep')
            

    def start(self):
        self.stop_event.clear()
        self.thread = threading.Thread(target=self.thread_func, args=())
        self.thread.start()

    def stop(self):
        self.stop_event.set()

    def thread_func(self):
        while not self.stop_event.is_set():
            raw_data = self.sock.recv(1024)
            # print('recv done')
            # decode received data into interpretable data type (string)
            decoded_data = raw_data.decode()
            self.decoded_data = decoded_data


if __name__ == '__main__':
    glove = SensorGlove()
    glove.calibrate_imu(10)
    glove.start()
    # q = multiprocessing.Queue()
    # p = multiprocessing.Process(target=glove.process, args=(q,))
    # p.start()
    start = time.time()
    while time.time() - start < 10:
        print(glove.decoded_data)
        glove.get_data()
    # glove.stop()
    # p.join()
    # glove.start()
    # time.sleep(10)
    glove.stop()
    time.sleep(1)
