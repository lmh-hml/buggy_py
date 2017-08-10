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
    """Loads poses from a file into a PoseAndPath object.
    When loaded by rosparam.load,the output variable 'loaded' should be arranged as so:
    [ (  { 'date':'...' , 'src_frame': '...', 'Poses' : [...] , 'how_many': ... } , 'map':... ) ]
    """
    try:
        loaded = rosparam.load_file(filename);
        print loaded[0][0].keys()
        poses = loaded[0][0]["Poses"];
        num_poses = len(poses)
        if num_poses == 0:
            print "PATH LENGTH IS 0"
            return False
        else:
            print "Loading %d poses..."%num_poses
        for i in range(num_poses):
            pose = PoseStamped();
            header = Header();
            fillPose(pose, poses[i]["position"], poses[i]["orientation"], header);
            poseAndPath.append_path_array(pose);
        return True

    except rosparam.RosParamException as ex:
        print ex.message;
        return False


#RUN THIS BY ITSELF TO LOAD A PATH WITHOUT GUI
if __name__ == '__main__':

    rospy.init_node("load_path");
    file_dir = rospy.get_param("file_dir","/home/buggy/catkin_ws/src/buggy_py/txt/");
    filename = rospy.get_param("~file", "example.txt");
    pap = PoseAndPath("record","map","base_link");
    header = Header();
    header.frame_id = pap.src;
    success = False;

    success = load_poses_from_file(file_dir+filename, pap)

    if success == True:
        print "Publishing loaded path..."
        while not rospy.is_shutdown() :
            pap.publish();
    else:
        print "Failed to load path!"
    exit();
