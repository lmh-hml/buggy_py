#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf

if __name__ == '__main__':

    rospy.init_node("tf_node_py");
    tfListener = tf.TransformListener();

    mto_pose_pub = rospy.Publisher("mto_pose_py",PoseStamped,queue_size=10);
    otb_pose_pub = rospy.Publisher("otb_pose_py",PoseStamped,queue_size=10);
    mtb_pose_pub = rospy.Publisher("mtb_pose_py",PoseStamped,queue_size=10);

    mtb_path_pub = rospy.Publisher("mtb_path",Path,queue_size=10);

    mtb_path = Path();
    mtb_path.header.frame_id = "map";
    mtb_path.header.seq = 0;

    pathFile  = open("/home/buggy/catkin_ws/src/buggy_py/scripts/path.txt",'w');
    rate = rospy.Rate(1.0);
    seq = 0;

    while not rospy.is_shutdown():

        now = rospy.Time.now();
        rate.sleep();

        try:
            #local poses so that path gets deep copies instead of shallow copies.
            mto_pose = PoseStamped();
            mtb_pose = PoseStamped();

            rospy.loginfo("Attempting to listen to tf: \n"+str(seq));
            mto_pose.header.frame_id = "map";
            mto_pose.pose.position, mto_pose.pose.orientation = tfListener.lookupTransform("map","odom",rospy.Time(0));
            mto_pose.header.seq = seq;
            mto_pose.header.stamp = now;

            mtb_pose.header.frame_id = "map";
            mtb_pose.pose.position, mtb_pose.pose.orientation = tfListener.lookupTransform("map","base_link",rospy.Time(0));
            mtb_pose.header.seq = seq;
            mtb_pose.header.stamp = now;

            mto_pose_pub.publish(mto_pose);
            mtb_pose_pub.publish(mtb_pose);

            mtb_path.header.stamp = now;
            mtb_path.poses.append(mtb_pose);
            length  = len(mtb_path.poses);
            last = str(mtb_path.poses[length-1]);
            rospy.loginfo( "path: {0},{1}".format( length , last ) );

            seq+=1;

        except tf.LookupException as ex:
            rospy.logerr(ex.message);
            continue

    pathFile.writelines( str(len(mtb_path.poses)) +'\n');
    pathFile.write(str(mtb_path));
    pathFile.close();
    print "NODE END!"
