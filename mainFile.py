import cv2
import time
import playsound
import threading


def passed():
    playsound.playsound('mission_passed.mp3')

def failed():
    playsound.playsound('mission_failed.mp3')
class MotionDetector:
    def __init__(self, threshold=50):
        self.threshold = threshold
        self.previous_frame = None
        self.start_time = None

    def detect_motion(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        if self.previous_frame is None:
            self.previous_frame = gray
            self.start_time = time.time()
            return False

        frame_diff = cv2.absdiff(self.previous_frame, gray)
        thresholded = cv2.threshold(frame_diff, self.threshold, 255, cv2.THRESH_BINARY)[1]
        thresholded = cv2.dilate(thresholded, None, iterations=2)
        contours, hierarchy = cv2.findContours(thresholded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Motion detected, update previous frame and return True
            self.previous_frame = gray
            self.start_time = time.time()
            return True
        self.previous_frame = gray
        return False


md = MotionDetector()
cap = cv2.VideoCapture(0)

start_time = cv2.getTickCount()
countdown_time = 15
elapsed_time = 0
game_over = False
motiond = 0
end = 0

circle_color = (0, 255, 0)  # Green color
circle_radius = 50
circle_thickness = 5
circle_center = (100, 150)
last_color_change_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.resize(frame, (1280, 720))
    motion_detected = md.detect_motion(frame)
    current_time = time.time()
    if current_time - last_color_change_time > 3:
        last_color_change_time = current_time
        if circle_color == (0, 255, 0):
            circle_color = (0, 0, 255)  # Red color
        else:
            circle_color = (0, 255, 0)  # Green color

    cv2.circle(frame, circle_center, circle_radius, circle_color, circle_thickness)

    if not game_over:
        elapsed_time = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
        remaining_time = countdown_time - int(elapsed_time)
        if remaining_time <= 0 or motiond == 1:
            game_over = True
        else:
            cv2.putText(frame, "Time: {}".format(remaining_time), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            if cv2.waitKey(1) & 0xFF == ord('k'):

                def fun1():
                    cv2.putText(frame, "MISSION PASSED. RESPECT ++", (200, 360), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 4)
                    win_start_time = time.time()
                    while time.time() - win_start_time < 0.1:
                        cv2.imshow('Frame', frame)
                        cv2.waitKey(1)
                thread1 = threading.Thread(target=fun1())
                thread2 = threading.Thread(target=passed())
                thread1.start()
                thread2.start()
                break

    if motion_detected:
        if circle_color == (0, 0, 255):
            motiond = 1
        cv2.putText(frame, "Motion Detected", (1000, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    else:
        cv2.putText(frame, "Not Detected", (1000, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('Frame', frame)

    if game_over == True and end == 0:
        def f2():
            cv2.putText(frame, "WASTED", (480, 360), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 4)
            game_over_start_time = time.time()
            while time.time() - game_over_start_time < 0.1:
                cv2.imshow('Frame', frame)
                cv2.waitKey(1)
        thread1 = threading.Thread(target=f2())
        thread2 = threading.Thread(target=failed())
        thread1.start()
        thread2.start()
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
