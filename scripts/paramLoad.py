#!/usr/bin/env python

import rospy
import rosparam
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from collections import Iterable
from PoseAndPath import PoseAndPath, fillPose
from nav_msgs.msg import Path
import types

def load_poses_from_file(filename, poseAndPath):
    """Loads poses from a file into a PoseAndPath object."""
    try:
        loaded = rosparam.load_file(filename);
        print loaded[0][0].keys()
        poses = loaded[0][0]["Poses"];
        if poses.__class__ == types.NoneType:
            print "POSES HAVE NO POSE IN IT"
            return False
        num_poses = len(poses)
        print num_poses
        for i in range(num_poses):
            pose = PoseStamped();
            header = Header();
            fillPose(pose, poses[i]["position"], poses[i]["orientation"], header);
            poseAndPath.append_path_array(pose);
        return True

    except rosparam.RosParamException as ex:
        print ex.message;
        return False




if __name__ == '__main__':

    rospy.init_node("Record");
    file_dir = rospy.get_param("file_dir","/home/buggy/catkin_ws/src/buggy_py/txt/");
    filename = rospy.get_param("~file", "waypoint_path2.txt");
    pap = PoseAndPath("record","map","base_link");
    header = Header();
    header.frame_id = pap.src;
    success = False;

    success = load_poses_from_file(file_dir+filename, pap)

    if success == True:
        print "Publishing loaded path..."
        while not rospy.is_shutdown() :
            pap.publish();
