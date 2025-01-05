#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def attach_links(model_name_1, link_name_1, model_name_2, link_name_2):
    """
    This function creates a service proxy to the attach service of the gazebo_ros_link_attacher node
    and sends a request to attach two specified links.
    """
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Prepare the request
    req = AttachRequest()
    req.model_name_1 = model_name_1
    req.link_name_1 = link_name_1
    req.model_name_2 = model_name_2
    req.link_name_2 = link_name_2

    # Call the service
    response = attach_srv.call(req)
    
    return response  # You can return the response if needed
y