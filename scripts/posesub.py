#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import math

class twist_calc():

    last_pose = PoseStamped();
    current_pose = PoseStamped();

    def __init__(self):
        self.last_pose = PoseStamped();
        self.current_pose = PoseStamped();


    def poseCB(self,poseStamped):

        self.current_pose = poseStamped;

    def calc_twist(self):
        offsetX = self.current_pose.pose.position.x - self.last_pose.pose.position.x;
        offsetY = self.current_pose.pose.position.y - self.last_pose.pose.position.y;
        offset = math.sqrt( math.pow(offsetX,2) + math.pow(offsetY,2));
        time_diff = (self.current_pose.header.stamp - self.last_pose.header.stamp).to_sec();
        vel = 0.0;
        if time_diff != 0:
            vel = offset/time_diff;
        print str(offsetX) + "," + str(offsetY)+","+str(offset)+" vel: "+ str(vel);
        print str(time_diff);
        self.last_pose = self.current_pose;




if __name__ == '__main__':
    rospy.init_node("poseSub");
    twister = twist_calc();
    pose_sub = rospy.Subscriber("pose_stamped", PoseStamped,twister.poseCB);


    while not rospy.is_shutdown():
        print "running";
        twister.calc_twist();
        rospy.Rate(0.5).sleep();
