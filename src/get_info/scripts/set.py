#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
import math

def publish_static_transform(object_position, parent_frame, child_frame):
   
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = object_position.x
    transform.transform.translation.y = object_position.y
    transform.transform.translation.z = object_position.z
    
    transform.transform.rotation.x = 0
    transform.transform.rotation.y = 0
    transform.transform.rotation.z = 0
    transform.transform.rotation.w = 1
    
    
    broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('static_tf2_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
       
    
    object_position_x = rospy.get_param('/object_position/x')
    object_position_y = rospy.get_param('/object_position/y')
    object_position_z = rospy.get_param('/object_position/z')
    
    
    object_point_stamped = PointStamped()
    object_point_stamped.header.frame_id = "camera_frame"  
    object_point_stamped.header.stamp = rospy.Time.now()
    object_point_stamped.point.x = object_position_x
    object_point_stamped.point.y = object_position_y
    object_point_stamped.point.z = object_position_z
    
    
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    
    rospy.sleep(1)  
    try:
        
        transform = tf_buffer.lookup_transform('map', 'camera_frame', rospy.Time(0), rospy.Duration(3.0))
        object_position_map = tf2_geometry_msgs.do_transform_point(object_point_stamped, transform)
        
        publish_static_transform(object_position_map.point, 'map', 'object_frame')
        rospy.loginfo("Published static transform from 'map' to 'object_frame'")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr('Error getting transform from camera_frame to map: %s' % str(e))
    
    rospy.spin()  #
