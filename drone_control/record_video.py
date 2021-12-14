import time, cv2
from threading import Thread
from djitellopy import Tello

FPS = 30

tello = Tello()
tello.connect()
print("Battery level:", tello.get_battery())

keepRecording = True
tello.streamon()
frame_read = tello.get_frame_read()

def video_recorder():
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), FPS, (width, height))

    while keepRecording:
        start = time.time()
        video.write(frame_read.frame)
        time.sleep(1 / FPS - (time.time() - start))

    video.release()

# we need to run the recorder in a seperate thread, otherwise blocking options
#  would prevent frames from getting added to the video
recorder = Thread(target=video_recorder)
recorder.start()

# tello.takeoff()
# tello.move_up(100)
# tello.rotate_counter_clockwise(360)
# tello.land()
start = time.time()
while time.time() - start < 15:
    cv2.imshow("Drone", frame_read.frame)
    cv2.waitKey(1)

keepRecording = False
recorder.join()

tello.end()
