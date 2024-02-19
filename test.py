import cv2
import numpy as np
from PIL import ImageFont, ImageDraw, Image
cap = cv2.VideoCapture(0)
font = ImageFont.truetype('effect1.ttf', 32)
def add_text(frame, text, font, position=(0, 0), color=(255, 255, 255)):
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    draw.text(position, text, font=font, fill=color)
    frame = np.array(img)
    return frame

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = add_text(frame, 'Hello, world!', font)

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()