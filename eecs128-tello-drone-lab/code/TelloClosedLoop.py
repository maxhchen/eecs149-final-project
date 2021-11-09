# This script is part of our course on Tello drone programming
# https://learn.droneblocks.io/p/tello-drone-programming-with-python/

# Import the necessary modules
import socket
import threading
import time
from time import sleep
import sys
import numpy as np
from queue import Queue
from queue import LifoQueue


State_data_file_name = 'statedata.txt'
index = 0
reference = 0.0     # Reference signal
control_LR = 0      # Control input for left/right
control_FB = 0      # Cotnrol input for forward/back
control_UD = 0      # Control input for up/down
control_YA = 0      # Control input for yaw
INTERVAL = 0.05  # update rate for state information
start_time = time.time()
dataQ = Queue()
stateQ = LifoQueue() # have top element available for reading present state by control loop

# IP and port of Tello for commands
tello_address = ('192.168.10.1', 8889)
# IP and port of local computer
local_address = ('', 8889)
# Create a UDP connection that we'll send the command to
CmdSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
CmdSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Bind to the local address and port
CmdSock.bind(local_address)

###################
# socket for state information
local_port = 8890
StateSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
StateSock.bind(('', local_port))
CmdSock.sendto('command'.encode('utf-8'), tello_address)   # command port on Tello
    
def writeFileHeader(dataFileName):
    fileout = open(dataFileName,'w')
    #write out parameters in format which can be imported to Excel
    today = time.localtime()
    date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
    date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
    fileout.write('"Data file recorded ' + date + '"\n')
    # header information
    fileout.write('  index,   time,    ref,ctrl_LR,ctrl_FB,ctrl_UD,ctrl_YA,  pitch,   roll,    yaw,    vgx,    vgy,    vgz,   templ,   temph,    tof,      h,    bat,   baro,   time,    agx,    agy,    agz\n\r')
    fileout.close()


def writeDataFile(dataFileName):
    fileout = open(State_data_file_name, 'a')  # append
    print('writing data to file')
    while not dataQ.empty():
        telemdata = dataQ.get()
        np.savetxt(fileout , [telemdata], fmt='%7.3f', delimiter = ',')  # need to make telemdata a list
    fileout.close()



def report(str,index):
    telemdata=[]
    telemdata.append(index)
    telemdata.append(time.time()-start_time)
    telemdata.append(reference)
    telemdata.append(control_LR)
    telemdata.append(control_FB)
    telemdata.append(control_UD)
    telemdata.append(control_YA)
    data = str.split(';')
    data.pop() # get rid of last element, which is \\r\\n
    for value in data:
        temp = value.split(':')
        if temp[0] == 'mpry': # roll/pitch/yaw
            temp1 = temp[1].split(',')
            telemdata.append(float(temp1[0]))     # roll
            telemdata.append(float(temp1[1]))     # pitch
            telemdata.append(float(temp1[2]))     # yaw
            continue
        quantity = float(value.split(':')[1])
        telemdata.append(quantity)
    dataQ.put(telemdata)
    stateQ.put(telemdata)
    if (index %100) == 0:
        print(index, end=',')
 
    

# Send the message to Tello and allow for a delay in seconds
def send(message):
  # Try to send the message otherwise print the exception
  try:
    CmdSock.sendto(message.encode(), tello_address)
    # print("Sending message: " + message)
  except Exception as e:
    print("Error sending: " + str(e))

# receive state message from Tello
def rcvstate():
    print('Started rcvstate thread')
    index = 0
    while not stateStop.is_set():
        
        response, ip = StateSock.recvfrom(1024)
        if response == 'ok':
            continue
        report(str(response),index)
        sleep(INTERVAL)
        index +=1
    print('finished rcvstate thread')

# Receive the message from Tello
def receive():
  # Continuously loop and listen for incoming messages
  while True:
    # Try to receive the message otherwise print the exception
    try:
      response, ip_address = CmdSock.recvfrom(128)
      print("Received message: " + response.decode(encoding='utf-8'))
    except Exception as e:
      # If there's an error close the socket and break out of the loop
      CmdSock.close()
      print("Error receiving: " + str(e))
      break

      

# Create and start a listening thread that runs in the background
# This utilizes our receive function and will continuously monitor for incoming messages
receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()

writeFileHeader(State_data_file_name)  # make sure file is created first so don't delay
stateThread = threading.Thread(target=rcvstate)
stateThread.daemon = False  # want clean file close
stateStop = threading.Event()
stateStop.clear()
stateThread.start()


# Tell the user what to do
print('Type in a Tello SDK command and press the enter key. Enter "quit" to exit this program.')

# Loop infinitely waiting for commands or until the user types quit or ctrl-c
while True:
  
  try:
    # Read keybord input from the user
    if (sys.version_info > (3, 0)):
      # Python 3 compatibility
      message = input('')
    else:
      # Python 2 compatibility
      message = raw_input('')
    
    # If user types quit then lets exit and close the socket
    if 'quit' in message:
      print("Program exited")
      CmdSock.close()  # sockete for commands
      stateStop.set()  # set stop variable
      stateThread.join()   # wait for termination of state thread before closing socket
      StateSock.close()  # socket for state  
      writeDataFile(State_data_file_name)
      print("sockets and threads closed")
      # send message
      
      break
    
    # Send the command to Tello
    send(message)
    sleep(10.0) # wait for takeoff and motors to spin up
    # height in centimeters
    print('takeoff done')
    
###################################################################################################
#################################DON'T TOUCH ANYTHING OUTSIDE THIS#################################
###################################################################################################
    
    # Controller Variables
    kp = 3 # <--------------------------------------------------------------------------------------- Fill this out
    
    # Control stores
    integratedError = 0.0
    errorStore = 0.0
    
    # Useful Reference Signal Variables
    period = 20 # <----------------------------------------------------------------------------------- Fill this out
    amplitude = 180 # <-------------------------------------------------------------------------------- Fill this out
    dt = 0.1
    
    # to prevent hickups
    lastTime = 0.0
    lastYaw = 0.0
    
    for i in range(0,500):
        
        # Get data (read sensors)
        presentState = stateQ.get(block=True, timeout=None)  # block if needed until new state is ready
        ptime = presentState[1]     # present time (don't over write time function)
        yaw = presentState[9]       # current yaw angle (don't overwrite)
        if lastTime > ptime:
            ptime = lastTime
            yaw = lastYaw
        
        # Compute Reference Signal (Triangle wave)
        reference = (2 * amplitude) / np.pi * np.arcsin(np.sin(2*np.pi * ptime / period))# <---------------------------------------------------------------------------- Fill this out
        
        # Compute Error and Control Input
        error = reference - yaw# <-------------------------------------------------------------------------------- Fill this out
        control_YA = kp * error # <--------------------------------------------------------------------------- Fill this out
        
        lastTime = ptime
        lastYaw = yaw
        
###################################################################################################
#################################DON'T TOUCH ANYTHING OUTSIDE THIS#################################
###################################################################################################
        
        
        # Send Control to quad
        control_LR = int(np.clip(-100,100,control_LR))
        control_FB = int(np.clip(-100,100,control_FB))
        control_UD = int(np.clip(-100,100,control_UD))
        control_YA = int(np.clip(-100,100,control_YA))
        message = 'rc '+str(control_LR)+' '+str(control_FB)+' '+str(control_UD)+' '+str(control_YA)
        #print(message)
        send(message)
        
        # Wait so make sample time steady
        sleep(0.1)
       
    message='rc 0 0 0 0' # stop motion
    control_input = 0
    send(message)
    sleep(1.5)
    message ='land'
    send(message)
    print('landing')
    
    # Handle ctrl-c case to quit and close the socket
  except KeyboardInterrupt as e:
    message='emergency' # try to turn off motors
    send(message)
    CmdSock.close()
    StateSock.close()  # socket for state
    stateStop.set()  # set stop variable
    writeDataFile(State_data_file_name)
    stateThread.join()   # wait for termination of state thread
    print("sockets and threads closed")
    break