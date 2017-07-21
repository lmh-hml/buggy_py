#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
import tf
import time

def fillPose( poseStamped, pos, orient, header ):
    poseStamped.header = header;
    poseStamped.pose.position.x = pos[0];
    poseStamped.pose.position.y = pos[1];
    poseStamped.pose.position.z = pos[2];
    poseStamped.pose.orientation.w = orient[0];
    poseStamped.pose.orientation.x = orient[1];
    poseStamped.pose.orientation.y = orient[2];
    poseStamped.pose.orientation.z = orient[3];

class PoseAndPath:

    def __init__(self,name,src_frame,dest_frame):
        self.name = name;
        self.src  = src_frame ;
        self.dest = dest_frame;
        self.header = Header();
        self.pose = PoseStamped();
        self.path = Path();
        self.pose_array = PoseArray();
        self.pose_pub = rospy.Publisher( name+"_pose_py",PoseStamped,queue_size=10);
        self.path_pub = rospy.Publisher(name+"_path_py",Path,queue_size=10);
        self.poseArray_pub = rospy.Publisher(name+"_poseArray_py",PoseArray,queue_size=10);
        self.header.frame_id = src_frame;

    def lookup_pose(self,tflistener, seq, now):
        pos , orient = tfListener.lookupTransform(self.dest,self.src,rospy.Time(0));
        self.pose = PoseStamped();
        self.header = Header();
        self.header.stamp = now;
        self.header.seq   = seq;
        fillPose(self.pose, pos,orient,self.header);

    def update_path_array(self):
        self.pose_array.poses.append(self.pose.pose);
        self.path.poses.append(self.pose);
        print str(self.pose);

    def publish(self):
        self.pose_pub.publish(self.pose);
        self.path_pub.publish(self.path);
        self.poseArray_pub.publish(self.pose_array);


    def reportThis(self):
        print str(self.header);
        print str(self.pose);
        print str(self.path);
        print str(self.pose_array);

    def write_path_to_file(self, filepath):
        pathFile  = open(filepath,'w');
        pathFile.writelines( time.ctime() +"\n"+str(len(self.path.poses)) +'\n');
        pathFile.write(str(self.path));
        pathFile.close();


if __name__ == '__main__':

    rospy.init_node("tf_node_py");
    tfListener = tf.TransformListener();

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
            mtb.update_path_array();
            mtb.publish();

            otb.lookup_pose(tfListener,seq,now);
            otb.update_path_array();
            otb.publish();

            length  = len(mtb.path.poses);
            last = str(mtb.path.poses[length-1]);
            rospy.loginfo( "path: {0},{1}".format( length , last ) );
            seq+=1;

        except tf.LookupException as ex:
            rospy.logerr(ex.message);
            continue

    mtb.write_path_to_file("/home/buggy/catkin_ws/src/buggy_py/scripts/mtb_path.txt");
    otb.write_path_to_file("/home/buggy/catkin_ws/src/buggy_py/scripts/otb_path.txt");

    print "NODE END!"
