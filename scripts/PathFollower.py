#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from PoseAndPath import PoseAndPath, EmptyPoseAndPathError
from Target_Pose import TargetPoser, GoalState
from tf.transformations import *
from GUI import GUI, PrintQueue,tk
import tkFileDialog as tkfile
from Queue import Queue
import threading
from enum import Enum
import exceptions
import sys

#Class used for storing a path and publishing poses from path to movebase.

class PathFollower:

    def __init__(self, nav_server="move_base", poseAndPath_name="waypoint",src_frame = "map", dest_frame = "base_link"):
        self.tp = TargetPoser(nav_server);
        self.pap = PoseAndPath( name=poseAndPath_name, src_frame=src_frame, dest_frame=dest_frame);
        self.state = GoalState(num=-1,msg="");
        self.mode = NavMode.IDLE;
        self.target = 0;
        self.begin =0;
        self.step = 1
        self.topic_name = "";
        self.begin_end = [PoseStamped(),PoseStamped()];

    def poseCB(self,poseStamped):
        self.pap.poseCB(poseStamped);
        self.update_begin_pose();

    def record_path(self,target_pose_topic):
        rospy.loginfo( "Subscribing to '%s'"%( target_pose_topic ) );
        self.sub = rospy.Subscriber(target_pose_topic,PoseStamped,self.poseCB);
        self.topic_name = target_pose_topic;

    def stop_recording(self):
        if self.topic_name != "":
            self.sub.unregister();
            rospy.loginfo("Unregistered from '%s'"%self.topic_name)

    def report_path_size(self):
        path_size = self.pap.getSize();
        return path_size;

    def update_state(self):
        self.state = self.tp.getState();

    def update_begin_pose(self):
        self.begin_end[0] = self.pap.getPoseStamped(self.begin);

    def report_state(self):
        """returns current actionlib client state as a string that indicates state id and msg, if any."""
        text = "Status: %d"%self.state.num;
        if self.state.msg !="":
            text += ", Msg: %s"%self.state.msg;
        return text;

    def follow_path_thread(self,size):
        running = True;
        self.target = self.begin;
        try:
            while running:
                rospy.loginfo("Going to Goal %d of %d"%(self.target,size));
                pose = self.pap.getPoseStamped(self.target);
                self.tp.send_pose_goal(pose);
                self.tp.movebase.wait_for_result();
                self.update_state();
                print("State: %d, msg: %s"%(self.state.num,self.state.msg));

                if self.state.num != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                    running = False;
                else:
                    self.target+=step;
                    if self.target>=size or self.target<0:
                        running=False;
                        print("Path finished!!!!");
                        self.target = 0;

        except Exception as e:
            print e.message;
        textQ.put("THREAD EXITS");

    def follow_path(self):
        size = self.report_path_size();
        if size <= 0:
            raise EmptyPoseAndPathError(self.pap.name);
        else:
            print "Thread!"
            try:
                t= threading.Thread(target=self.follow_path_thread, args=(size,));
                t.start();
            except Exception as e:
                self.error(e.message);

    def clearPath(self):
        self.begin = 0;
        self.target = 0;
        self.step = 1;
        if self.pap.getSize() > 0:
            del self.pap.path.poses[:];
            del self.pap.pose_array.poses[:];
        else:
            raise EmptyPoseAndPathError(self.pap.name);

    def remove_goal(self,index=None):
        if self.pap.getSize() > 0:
            if index != None:
                self.pap.path.poses.pop(index);
                self.pap.pose_array.poses.pop(index);
            else:
                self.pap.path.poses.pop();
                self.pap.pose_array.poses.pop();
        else:
            raise EmptyPoseAndPathError(self.pap.name);

if __name__ == '__main__':

    rospy.init_node("PathFollower");
    path_follower = PathFollower();
