import numpy as np
import cv2
import time

kernal = np.ones((5, 5), "uint8")

red_lower = np.array([136, 87, 111], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)
end = False
cons_confirm = 0

# Check for 3 neighbouring red rectangles and trace outline
def detect_rectangles(mask, frame, colour_name, colour_bgr):
    global end, cons_confirm
    
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)   
    blurred = cv2.GaussianBlur(closed, (5,5), 0)
    contours, _ =  cv2.findContours(blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    filled_mask = np.zeros_like(mask)
    cv2.drawContours(filled_mask, contours, -1, 255, thickness=cv2.FILLED)
    clean_contours, _ = cv2.findContours(filled_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rectangles = []
    grid = {
        "grid_1" : 0,
        "grid_2" : 0,
        "grid_3" : 0
    }

    for contour in clean_contours:
        area = cv2.contourArea(contour)

        if area>3000:
            rot_rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rot_rect)
            box = np.intp(box)

            lowest_y = max([point[1] for point in box])
            rectangles.append((lowest_y, box, rot_rect))
    
    # Sort list with nearest to bottom of frame first
    rectangles.sort(key=lambda x: -x[0])

    for rank, (lowest_y, box, rot_rect) in enumerate(rectangles, start=1):
        if len(rectangles) == 3:
            cv2.drawContours(frame, [box], 0, colour_bgr, 2)
                
            width, height = rot_rect[1]
            center_x, center_y = int(rot_rect[0][0]), int(rot_rect[0][1])
            distance_to_base = frame.shape[0] - center_y

            grid_name = f"grid_{rank}"
            grid[grid_name] = distance_to_base
            label = f"{colour_name} Rectangle {rank}: {distance_to_base}"
            cv2.putText(frame, label, (center_x-50, center_y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour_bgr, 2)

    # Check all values in dict not 0, only 5 consecutive detection will be confirmed
    results = all(value > 0 for value in grid.values())
    if results:
        cons_confirm += 1
        if cons_confirm >= 5:
            end = True
    else:
        cons_confirm = 0
    
    return grid

# Main function
def get_grid(frame):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define HSV color ranges (do not change)  
    # Create masks
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Dilate masks
    red_mask = cv2.dilate(red_mask, kernal)
    
    grid = detect_rectangles(red_mask, frame, "Red", (0,0,255))

    return frame, grid, end