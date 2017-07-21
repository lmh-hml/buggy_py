#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

    rospy.init_node("posePub");
    pub = rospy.Publisher("pose_stamped",PoseStamped, queue_size=10);
    ppose = PoseStamped();
    x =0.0;
    y= 0.0;


    while not rospy.is_shutdown():
        ppose.pose.position.x = x;
        ppose.pose.position.y = y;
        ppose.header.stamp = rospy.Time.now()
        ppose.header.frame_id = "map";
        rospy.loginfo( "publishing " + str(ppose));
        pub.publish(ppose);

        x +=1;
        y +=1;
        rospy.Rate(1.0).sleep();
