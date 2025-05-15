import cv2
import numpy as np
import time
import math

import detect_assembly

min_area = 9000
image_center = (320, 240)

def mouse_callback(event, x, y, flags, param):
    if event==cv2.EVENT_LBUTTONDOWN:
        hsv = param
        hsv_pixel = hsv[y,x]
        print(f"HSV at {x}, {y}: {hsv_pixel}")

def detect_box(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
       
    lower_green = np.array([50,120,30])

    upper_green = np.array([85,255,230])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    cv2.imshow("Mask", mask)

    contours, _=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    valid_contours = []
    angle_x, angle_y, w, h = None, None, 0, 0

    for contour in contours:
        area = cv2.contourArea(contour)

        if area > min_area:
            epsilon = 0.02*cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) >= 4:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w)/h
            
                if 0.5 <= aspect_ratio <= 2:
                    valid_contours.append((contour, area))

    if valid_contours:
        largest_contour, _ = max(valid_contours, key=lambda x:x[1])
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
        cv2.putText(frame, "Green Object", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # Mark center
        center_x = (x+w)//2
        center_y = (y+h)//2
        cv2.circle(frame, (center_x, center_y), 2, (0,0,255), 3)

        angle_x, angle_y = detect_assembly.pixel2angle(center_x, center_y)
        cv2.putText(frame, f"Angle x: {angle_x:.2f}", (x-50, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.putText(frame, f"Angle y: {angle_y:.2f}", (x-50, y+40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)  

        
        # if position == "position":
        #     if angle_x < -1:
        #         if w > h:
        #             position = "horizontal"
        #             print(position)
        #         else:
        #             position = "vertical"
        #             print(position)

    return frame, hsv, angle_x, angle_y, w, h