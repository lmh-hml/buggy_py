#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from PoseAndPath import PoseAndPath as PAP
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

    rospy.init_node("waypoint");

    target_pose_topic = rospy.get_param("~target_pose","target_pose");
    pap = PAP("waypoint","map","base_link");

    rospy.loginfo( "Subscribing to %s"%( target_pose_topic ) );
    rospy.Subscriber(target_pose_topic,PoseStamped,pap.poseCB);

    while not rospy.is_shutdown():
        rospy.spin();

    pap.write_path_to_file("/home/buggy/catkin_ws/src/buggy_py/txt/waypoint_path.txt");
