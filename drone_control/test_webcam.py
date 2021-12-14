import cv2
from collections import deque
from time import sleep, time
import threading


class CamCapture:
    def __init__(self, camID, buffer_size):
        self.queue = deque(maxlen=buffer_size)
        self.status = False
        self.isstop = False
        self.capture = cv2.VideoCapture(camID, cv2.CAP_V4L2)
        ret = self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if not ret:
            raise RuntimeError("Couldn't set fourcc")
        ret = self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not ret:
            raise RuntimeError("Couldn't set buffer size")


    def start(self):
        print('camera started!')
        t1 = threading.Thread(target=self.queryframe, daemon=True, args=())
        t1.start()

    def stop(self):
        self.isstop = True
        print('camera stopped!')

    def getframe(self):
        if len(self.queue) == 0:
            return None
        print('current buffers : ', len(self.queue))
        return self.queue.popleft()

    def queryframe(self):
        while (not self.isstop):
            start = time()
            self.status, tmp = self.capture.read()
            self.queue.append(tmp[:, ::-1]) # Flips image
            print('read frame processed : ', (time() - start) *1000, 'ms')

        self.capture.release()

cam = CamCapture(camID=0, buffer_size=50)
W, H = 1280, 720
cam.capture.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cam.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
print("Camera FPS:", cam.capture.get(cv2.CAP_PROP_FPS))

# start the reading frame thread
cam.start()
sleep(.1)

frames = 0
start = time()

while True:
  frame = cam.getframe() # numpy array shape (720, 1280, 3)
  if frame is None:
      continue

  cv2.imshow('video',frame)

  frames += 1
  print(f"FPS: {frames / (time() - start)}")

  if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        cam.stop()
        break
