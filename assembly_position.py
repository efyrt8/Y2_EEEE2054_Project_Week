import cv2
import numpy as np
import math
from collections import Counter

# Define the center of the image
image_center = (320, 240)
frame_height = 480

def detect_assembly_position(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Max label value and distances array
    max_label = 0
    distance_to_bottom = []
    
    # Detect circles
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=50,
        param1=120,
        param2=70,
        minRadius=10,
        maxRadius=200
    )

    if circles is not None:
        circles = np.uint16(np.around(circles)) 

        grouped_circles = {}
        frame_height = frame.shape[0]

        for circle in circles[0, :]:
            x, y, r = circle
            dist_to_bottom_px = frame_height - y
            bucket_key = round(dist_to_bottom_px / 5)*5

            if bucket_key not in grouped_circles or r < grouped_circles[bucket_key][2]:
                grouped_circles[bucket_key] = (x, y, r)
        
        circle_distances = [(x, y, r, frame_height - y) for (x, y, r) in grouped_circles.values()]
        circle_distances.sort(key=lambda c: c[3])
        distance_to_bottom = [c[3] for c in circle_distances]

        for idx, (x, y, r, dist_to_bottom_px) in enumerate(circle_distances):
            # Distance from circle center to bottom of frame
            label = idx + 1
            max_label = label

            # Draw the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

            # Display information
            cv2.putText(frame, f"Dist to Bottom: {dist_to_bottom_px:.0f} px", (x - 50, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Label the circle
            cv2.putText(frame, f"{idx + 1}", (x + r + 5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    # Draw the center of the frame
    cv2.circle(frame, image_center, 5, (255, 0, 0), 2)
    cv2.line(frame, (image_center[0] - 10, image_center[1]), (image_center[0] + 10, image_center[1]), (255, 0, 0), 2)
    cv2.line(frame, (image_center[0], image_center[1] - 10), (image_center[0], image_center[1] + 10), (255, 0, 0), 2)

    return frame, max_label, distance_to_bottom

# Compare obtained results
def compare_results(max_label_record, distance_record, best_ind):
    # Find most common label
    label_counter = Counter(max_label_record)
    most_common, _ = label_counter.most_common(1)[0]

    all_index = [
        i for i, label in enumerate(max_label_record) 
        if label == most_common
    ]
    
    best_ind = all_index[0]
    for i in all_index[1:]:
        if len(distance_record[i]) > 0 and len(distance_record[best_ind]) > 0:
            if distance_record[i][0] > distance_record[best_ind][0]:
                best_ind = i
    
    final_ind = max_label_record[best_ind]
    final_dist = distance_record[best_ind]
    
    return final_ind, final_dist

def locate_grid(final_ind, final_dist, grid):
    position = -1
    result = -1

    if len(final_dist) == final_ind:
        position = final_dist[0]
        # for i in range (1, len(grid)+1):
        #     key = f"grid_{i}"

        #     if i == 1:
        #         if position < grid[key]:
        #             result = key
        #             break
        #     elif i == 2:
        #         prev_key = f"grid_{i-1}"
        #         if grid[prev_key] <= position < grid[key]:
        #             result = key
        #             break
        #     else:
        #         if position >= grid[key]:
        #             result = key
        #             break
        if position < grid["grid_1"]:
            result = "grid_1"
        elif grid["grid_1"] <= position < grid["grid_2"]:
            result = "grid_2"
        else:
            result = "grid_3"
    # if result == -1:
    #     result = "grid_2"
    print(f"result: {result}")
    return position, result