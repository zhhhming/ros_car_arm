#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def detach_links(model_name_1, link_name_1, model_name_2, link_name_2):
    """
    Function to detach two links in Gazebo using the gazebo_ros_link_attacher service.
    """
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    rospy.loginfo("Detaching {} and {}".format(model_name_1, model_name_2))
    req = AttachRequest()
    req.model_name_1 = model_name_1
    req.link_name_1 = link_name_1
    req.model_name_2 = model_name_2
    req.link_name_2 = link_name_2

    detach_srv.call(req)

if __name__ == '__main__':
    detach_links("unit_cylinder", "link", "robot", "right_gripper")