#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import geometry_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math

def calculate_goal_position_orientation(object_transform_map, offset_distance):
    
    goal_position = geometry_msgs.msg.Point()
    goal_position.x = object_transform_map.transform.translation.x
    goal_position.y = object_transform_map.transform.translation.y
    goal_position.z = 0  

    
    
    direction_vector = [goal_position.x, goal_position.y]
    yaw = math.atan2(direction_vector[1], direction_vector[0])
    
  
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

    
    goal_position.x -= offset_distance * math.cos(yaw)
    goal_position.y -= offset_distance * math.sin(yaw)

    
    orientation = geometry_msgs.msg.Quaternion(*quaternion)
    
    return goal_position, orientation

def main():
    rospy.init_node('move_base_goal_setter')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    goal_sent = False
    while not rospy.is_shutdown() and not goal_sent:
        object_transform_map = tf_buffer.lookup_transform('map', 'object_frame', rospy.Time(0), rospy.Duration(1.0))
        if object_transform_map:
            goal_position, orientation = calculate_goal_position_orientation(object_transform_map, 0.3)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = goal_position
            goal.target_pose.pose.orientation = orientation

            move_base_client.send_goal(goal)
            wait = move_base_client.wait_for_result()

            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                
                result = move_base_client.get_result()
                rospy.loginfo("Result: {}".format(result))
            goal_sent = True
        rospy.sleep(1)  

if __name__ == '__main__':
    main()
