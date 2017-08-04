#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,Twist,Quaternion
from sensor_msgs.msg import Joy
import tf.transformations as trf
from math import cos,sin,tan
from GUI import GUI

jX = 0.0
jY= 0.0
jQ =0.0
quat = [0.0,0.0,0.0,0.0]
pose = PoseStamped();
PI = 3.14159265
ONEEIGHTYOVERPI = 180/PI;
PIOVERONEEIGHT = PI/180;
MAG = 1.0

class JoyPose(PoseStamped):

    def __init__(self):
        PoseStamped.__init__(self);
        self.vx = 0.0
        self.vy = 0.0
        self.header.frame_id="map"

    def getQuatList(self):
        q = QuatToList(pose.pose.orientation);
        return q;

    def orientation_in_rad(self,which="roll"):
        q = QuatToList(self.pose.orientation);
        e = trf.euler_from_quaternion(q);
        return e

    def translate(self, mag, angle):
        vx = cos(angle);
        vy = sin(angle);
        self.pose.position.x -= vx*mag;
        self.pose.position.y += vy*mag;

    def rotate(self, angle, which="roll"):
        rads = self.orientation_in_rad(which);
        quat = trf.quaternion_from_euler(ak=0,aj=0,ai=(rads[0]+angle) )
        poseQuat(self.pose,quat);
        print rads

    def joyCB(self,joy):
        jY = joy.axes[1]
        jQ = -(PI/10.0)*joy.axes[3]

        rad = self.orientation_in_rad(which="roll");
        self.rotate(jQ,"roll");
        self.translate(jY,angle=rad[0])

        print "%f,%f"%(self.vx,self.vy)
        print rad
        print self

def rotateQuat( pose, angle, which="roll"):
    quat = pose.orientation
    qlist = [ quat.w,quat.x,quat.y,quat.z ];
    elist = trf.euler_from_quaternion(qlist);
    newQuat = trf.quaternion_from_euler(ak=0,aj=0,ai=(elist[0]+angle));
    poseQuat(pose,newQuat);


def poseQuat(pose,quat):
    pose.orientation.w = quat[0]
    pose.orientation.x = quat[1]
    pose.orientation.y = quat[2]
    pose.orientation.z = quat[3]

def QuatToList( quat):
    return [ quat.w,quat.x,quat.y,quat.z ]

def joy_cb(joy):
    global jX , jY, jQ
    jY = joy.axes[1]
    jQ = -(PI/10.0)*joy.axes[3]

    q = QuatToList(pose.pose.orientation);
    e = trf.euler_from_quaternion(q);
    quat  = trf.quaternion_from_euler(ak=0,aj=0,ai= e[0]+jQ )

    vx = cos(e[0]);
    vy = sin(e[0]);
    pose.pose.position.x -= vx*jY
    pose.pose.position.y += vy*jY
    poseQuat(pose.pose,quat)

    #print "%f,%f"%(vx,vy)
    #print e
    #print pose

if __name__ == '__main__':
    rospy.init_node("Joy_pose");
    pose.header.frame_id = "map"
    last = rospy.Time.now()
    jp = JoyPose();
    sub = rospy.Subscriber("joy",Joy,jp.joyCB);
    pub = rospy.Publisher("pose_stamped", PoseStamped, queue_size=10)

    def flip():
        rad = 180* PIOVERONEEIGHT;
        jp.rotate(angle=rad)

    gui = GUI();
    gui.addButton("Flip",flip);

    while not rospy.is_shutdown():
        gui.update()
        now = rospy.Time.now()
        elp = now - last;
        elp = elp.to_sec()
        pub.publish(jp)
