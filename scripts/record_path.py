#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from PoseAndPath import PoseAndPath as PAP
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

    rospy.init_node("record");

    target_pose_topic = rospy.get_param("~target_pose","target_pose");
    file_dir = rospy.get_param("file_dir","/home/buggy/catkin_ws/src/buggy_py/txt/");
    filename = rospy.get_param("~file", "waypoint_path2.txt");
    output_file = file_dir + filename;

    pap = PAP("waypoint","map","base_link");

    rospy.loginfo( "Subscribing to %s"%( target_pose_topic ) );
    rospy.Subscriber(target_pose_topic,PoseStamped,pap.poseCB);

    while not rospy.is_shutdown():
        rospy.spin();

    pap.write_path_simple(output_file);
