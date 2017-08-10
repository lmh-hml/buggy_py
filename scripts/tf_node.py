#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
import tf
import time
from PoseAndPath import PoseAndPath;
from PoseAndPath import fillPose;
from std_msgs.msg import Bool

class flag:
    def __init__(self,bool_val):
        self.val = bool_val

if __name__ == '__main__':

    rospy.init_node("tf_node_py");
    auto_record= rospy.get_param("~autoRecord",True);
    tfListener = tf.TransformListener();
    should_record = flag(auto_record);

    def boolCB(b):
        should_record.val = b.data;
        print should_record.val


    bool_sub = rospy.Subscriber("should_record", Bool,callback=boolCB, queue_size=100);

    otb = PoseAndPath("otb","odom","base_link");
    mto = PoseAndPath("mto","map","odom");
    mtb = PoseAndPath("mtb","map","base_link");

    rate_time = rospy.get_param("~rate",1.0);
    rate = rospy.Rate(1.0/rate_time);
    seq = 0;

    rospy.loginfo("rate @"+str(rate_time));

    while not rospy.is_shutdown():

        now = rospy.Time.now();
        rate.sleep();

        try:
            #local poses so that path gets deep copies instead of shallow copies.
            mto_pose = PoseStamped();
            mtb_pose = PoseStamped();
            otb_pose = PoseStamped();

            rospy.loginfo("Attempting to listen to tf: \n"+str(seq));

            pos , orient = tfListener.lookupTransform(mto.dest,mto.src,rospy.Time(0));
            fillPose(mto_pose, pos,orient,mto.header);
            mto.header.stamp = now;
            mto.header.seq   = seq;
            mto.pose_pub.publish(mto_pose);

            mtb.lookup_pose(tfListener,seq,now);
            otb.lookup_pose(tfListener,seq,now);

            if not should_record.val :
                print "Rec:"+str(should_record.val)
                continue
            rospy.loginfo("Recording");
            mtb.append_path_array(mtb.pose);
            mtb.publish();
            otb.append_path_array(pose=otb.pose);
            otb.publish();


            length  = len(mtb.path.poses);
            if length <= 0:
                continue
            last = str(mtb.path.poses[length-1]);
            rospy.loginfo( "path: {0},{1}".format( length , last ) );
            seq+=1;

        except tf.LookupException as ex:
            rospy.logerr(ex.message);
            continue

    mtb.write_path_simple("/home/buggy/catkin_ws/src/buggy_py/txt/mtb_path.txt");
    otb.write_path_simple("/home/buggy/catkin_ws/src/buggy_py/txt/otb_path.txt");

    print "NODE END!"
