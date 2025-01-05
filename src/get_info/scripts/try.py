#!/usr/bin/env python3

import rospy

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
def point_cloud_callback(point_cloud):
   
    point_index = 257*480
    total_points = point_cloud.width * point_cloud.height
    print(total_points)
  
    generator = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=False)

    print("begin")

    for _ in range(point_index):
        next(generator)

    point = next(generator)
    print("The coordinates of the point are: x={}, y={}, z={}".format(*point))

rospy.init_node('point_cloud_listener')


rospy.Subscriber("/camera/depth/points", PointCloud2, point_cloud_callback)


rospy.spin()
