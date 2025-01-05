import cv2
import numpy as np

def detect_white_object_center(cv_image):
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    
    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([180, 25, 255], dtype=np.uint8)

    
    mask = cv2.inRange(hsv, lower_white, upper_white)

    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            return cX, cY
    return None
import sensor_msgs.point_cloud2 as pc2

def get_object_position_from_pointcloud(pointcloud_data, pixel_x, pixel_y):
    
    point_list = pc2.read_points(pointcloud_data, field_names=("x", "y", "z"), skip_nans=False)

    
    number = (pixel_y-1)*640+pixel_x

    
    for _ in range(number-1):  
        try:
            next(point_list)
        except StopIteration:
            
            return None

    
    try:
        point = next(point_list)
        print("The coordinates of the point are: x={}, y={}, z={}".format(*point))
        return point  
    except StopIteration:
        
        return None
