# import bluetooth
# # 68:9A:87:CA:C3:08	
# bd_addr = "68:9A:87:CA:C3:08"
# port = 5
# sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
# sock.connect((bd_addr, port))
# sock.send("hello!!")
# sock.close()
from serial import * 
import time
arduino = Serial(port='COM5', baudrate=115200, timeout=.1)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data
while True:
    num = input("Enter a number: ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value