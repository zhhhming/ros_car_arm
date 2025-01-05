#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading
from color_detector import detect_white_object_center, get_object_position_from_pointcloud
from geometry_msgs.msg import PointStamped

# Initialize the bridge between ROS and OpenCV
bridge = CvBridge()

def process_data(rgb_image, pointcloud_data):
    try:
        
        cv_rgb_image = bridge.imgmsg_to_cv2(rgb_image, "bgr8")

        
        center_pixel = detect_white_object_center(cv_rgb_image)
        print(center_pixel[0],"ss",center_pixel[1])
        if center_pixel is None:
            print("No white object detected in the RGB image.")
            return
        position = get_object_position_from_pointcloud(pointcloud_data, center_pixel[0], center_pixel[1])
        if position:
            x, y, z = position
            rospy.set_param('object_position', {'x': x, 'y': y, 'z': z})
            
        else:
            print("Could not find the point in the point cloud.")
    except CvBridgeError as e:
        print(e)
    except Exception as ex:
        print("Error processing data:", ex)

def rgb_callback(rgb_data):
    global last_rgb_image
    last_rgb_image = rgb_data

def pointcloud_callback(pointcloud_data):
    global last_pointcloud_data
    last_pointcloud_data = pointcloud_data

def input_thread():
    global last_rgb_image, last_pointcloud_data
    while not rospy.is_shutdown():
        if input("Press 's' to process images: ") == 's':
            if last_rgb_image is not None and last_pointcloud_data is not None:
                process_data(last_rgb_image, last_pointcloud_data)

def listener():
    rospy.init_node('image_processor', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_callback)
    rospy.Subscriber("/camera/depth/points", PointCloud2, pointcloud_callback)
    print("Subscribed to /camera/rgb/image_raw and /camera/depth/points topics.")
    print("Press 's' to process the current images.")

    
    thread = threading.Thread(target=input_thread)
    thread.start()

    
    rospy.spin()

if __name__ == '__main__':
    last_rgb_image = None
    last_pointcloud_data = None
    listener()
