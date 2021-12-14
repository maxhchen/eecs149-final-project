import cv2
import numpy as np
import mediapipe as mp
import time

class HandDetector():
    def __init__(self, static_mode = False, maxHands = 2, complexity=1, detectionCon = 0.5, trackCon = 0.5):
        self.static_mode = static_mode
        self.maxHands = maxHands
        self.complexity = complexity
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.static_mode, 
                                        max_num_hands=self.maxHands, 
                                        model_complexity=self.complexity,
                                        min_detection_confidence=self.detectionCon, 
                                        min_tracking_confidence=self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

        self.landmarks = None
        
    def findHands(self, img, draw = True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo = 0, draw = True):
        lmlist = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmlist.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 3, (255, 0, 255), cv2.FILLED)
        self.landmarks = lmlist
        return lmlist

    def findCenter(self, img, lmlist, draw=True):
        center_x, center_y = None, None
        if len(lmlist) != 0:
            center_x = 0
            center_y = 0
            for lm in lmlist:
                center_x += lm[1]
                center_y += lm[2]
            center_x = int(center_x / len(lmlist))
            center_y = int(center_y / len(lmlist))
            if draw:
                cv2.circle(img, (center_x, center_y), 10, (0, 255, 0), cv2.FILLED)
        return (center_x, center_y)


def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    detector = HandDetector(static_mode=False, complexity=1)

    while True:
        success, img = cap.read()
        img = np.array(img[:, ::-1]) # Flip
        img = detector.findHands(img)
        lmlist = detector.findPosition(img, draw=False)
        center = detector.findCenter(img, lmlist)

        image_center = np.array(img.shape[:2]) / 2
        print(f"Hand Center: {center}, Image center: {image_center}")

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        cv2.putText(img, str(round(fps, 2)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

        cv2.imshow("Image", img)
        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == '__main__':
    main()
