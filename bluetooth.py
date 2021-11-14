"""
A simple Python script to send messages to a sever over Bluetooth using
Python sockets (with Python 3.3 or above).


Board: 68-9A-87-CA-C3-08	

Mine: DC-A9-04-80-95-B7 
"""
import socket
serverMACAddress = '68:9a:87:ca:c3:08'
port = "/dev/cu.usbserial-014AD0FD"
s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s.connect((serverMACAddress,port))
while 1:
    text = input()
    if text == "quit":
        break
    s.send(bytes(text, 'UTF-8'))
s.close()