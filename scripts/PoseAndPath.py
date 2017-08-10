#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
import tf
import time
import exceptions
from util import fillPose

#A class used to subscribe to a posestamped topic and collect received poses into a path and pose array .

class EmptyPoseAndPathError(Exception):
    def __init__(self, name=""):
        self.message = name+": member Path and Pose_array are both empty!!"

class PoseAndPath:

    def __init__(self,name,src_frame,dest_frame):
        self.name = name;
        self.src  = src_frame ;
        self.dest = dest_frame;
        self.header = Header();
        self.pose = PoseStamped();
        self.path = Path();
        self.pose_array = PoseArray();
        self.pose_pub = rospy.Publisher( name+"_pose",PoseStamped,queue_size=10);
        self.path_pub = rospy.Publisher(name+"_path",Path,queue_size=10);
        self.poseArray_pub = rospy.Publisher(name+"_poseArray",PoseArray,queue_size=10);
        self.header.frame_id = src_frame;
        self.path.header = self.header;
        self.pose_array.header = self.header;

    def poseCB(self,posestamped):
        self.pose = posestamped;
        self.append_path_array(self.pose);
        self.publish();

    #lookup a pose with a tflistener, and then fills in member pose with the result
    def lookup_pose(self, tfListener, seq, now):
        pos , orient = tfListener.lookupTransform(self.src,self.dest,rospy.Time(0));
        self.pose = PoseStamped();
        self.header = Header();
        self.header.frame_id = self.src;
        self.header.stamp = now;
        self.header.seq   = seq;
        fillPose(self.pose, pos,orient,self.header);

    def append_path_array(self,pose):
        self.pose_array.poses.append(pose.pose);
        self.path.poses.append(pose);

    #publishes member path and pose array. useful for visualization
    def publish(self):
        self.path_pub.publish(self.path);
        self.poseArray_pub.publish(self.pose_array);

    def getPose(self,index):
        try:
            pose = self.pose_array.poses[index];
            return pose;
        except IndexError as ex:
            print ex.message;

    def getPoseStamped(self,index):
        try:
            pose = self.path.poses[index];
            return pose;
        except IndexError as ex:
            print ex.message;

    def getSize(self):
        return min( len(self.path.poses),len(self.pose_array.poses) );

    #writes a path's yaml representation to a file
    def write_path_to_file(self, filepath):
        pathFile  = open(filepath,'w');
        pathFile.writelines( "date: " + time.ctime() +'\n');
        pathFile.writelines("pose_no: "+str( len(self.path.poses) ) + '\n' );
        pathFile.writelines("path : "+str(self.path));
        pathFile.close();

    #writes a condesed version of the path to a file.
    def write_path_simple(self,filepath, map=""):
        with open(filepath,'w') as pathfile:
            pathfile.writelines( "date: " + time.ctime() +'\n');
            length = len(self.path.poses);
            pathfile.writelines("how_many: "+str( length ) + '\n' );
            pathfile.writelines("src_frame: "+self.src + '\n' );
            pathfile.writelines("map: "+map+'\n');
            pathfile.writelines("\n\n" );
            pathfile.write("Poses: [\n");
            for i in range(length):
                pose = self.path.poses[i];
                pathfile.write('{\n');
                pathfile.write("    position: [%f,%f,%f],\n"%(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z) );
                pathfile.write("    orientation: [%f,%f,%f,%f]\n"%(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w) );
                pathfile.write("},\n");
            pathfile.write("\n]");
