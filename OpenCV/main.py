import cv2
import numpy as np
import serial
import ctypes
import struct
from matplotlib import pyplot as plt 

ser = serial.Serial("COM3", 115200, 8, "N", 1)
lower_range = np.array([0, 100, 100])
upper_range = np.array([10, 255, 255])
sample = 5
cur_sample = 0

center_x = 0
center_y = 0
capture = cv2.VideoCapture(0)
width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
refrence_x = int(width // 2)
refrence_y = int(height // 2)

while True:
    ret, frame = capture.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame = cv2.blur(frame, (9,9))
    frame = cv2.medianBlur(frame, 9)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_range, upper_range)
    img = cv2.bitwise_and(frame, frame, mask=mask)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, (3, 3), iterations=10)
    img = cv2.Canny(img, 127, 255)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, (3, 3), iterations=30)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
         # 计算中心坐标
        if M["m00"] != 0:  # 避免除以0

            center_x += int(M["m10"] / M["m00"])
            center_y += int(M["m01"] / M["m00"])
            cur_sample += 1

            if (cur_sample >= sample):
                cur_sample = 0
                center_x = int(center_x / sample)
                center_y = int(center_y / sample)
                err_x = int(refrence_x - center_x)
                err_y = int(refrence_y - center_y)

                print(f"Reference Point: ({refrence_x},{refrence_y}), Contour Center: ({center_x}, {center_y}), \
                    Error: ({err_x},{err_y})")
                data = struct.pack("<2B4H2B", 0xAE, 0xEA, refrence_x, refrence_y, center_x, center_y, 0xEA, 0xAE)
                ser.write(data)

        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)

    cv2.imshow("ori", frame)
    if cv2.waitKey(1) == ord('q'):
        break