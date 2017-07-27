#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import math

class twist_calc():

    def __init__(self):
        self.last_pose = PoseStamped();
        self.current_pose = PoseStamped();
        self.cmd_vel = Twist();

    def poseCB(self,poseStamped):

        self.current_pose = poseStamped;

    def twistCB(self,cmd_vel):

        self.cmd_vel = cmd_vel;

    def calc_twist(self):
        offsetX = self.current_pose.pose.position.x - self.last_pose.pose.position.x;
        offsetY = self.current_pose.pose.position.y - self.last_pose.pose.position.y;
        offset = math.sqrt( math.pow(offsetX,2) + math.pow(offsetY,2));
        time_diff = (self.current_pose.header.stamp - self.last_pose.header.stamp).to_sec();
        vel = 0.0;
        if time_diff == 0:
            return;

        vel = offset/time_diff;
        print("dX: %.3f, dY: %.3f, dV: %.3f, V:%.3f" % (offsetX, offsetY, offset, vel));
        print("@Cmd_vel: lx:%.3f, az:%.3f, over %.3f secs"%(self.cmd_vel.linear.x, self.cmd_vel.angular.z,time_diff));
        self.last_pose = self.current_pose;


if __name__ == '__main__':
    rospy.init_node("poseSub");
    twister = twist_calc();
    secs_per_sample = rospy.get_param("~rate",2.0);
    rate = 1.0/secs_per_sample;

    pose_sub = rospy.Subscriber("pose_stamped", PoseStamped,twister.poseCB);
    cmd_sub  = rospy.Subscriber("cmd_vel_out",  Twist, twister.twistCB);

    while not rospy.is_shutdown():
        print "running";
        twister.calc_twist();
        rospy.Rate(rate).sleep();
