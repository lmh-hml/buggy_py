#!/usr/bin/env python
PI = 3.14159265
ONEEIGHTYOVERPI = 180/PI;
PIOVERONEEIGHT = PI/180;

def degToRad(deg):
    return deg*PIOVERONEEIGHT

def radToDeg(rad):
    return rad*ONEEIGHTYOVERPI;


from geometry_msgs.msg import PoseStamped
import tf.transformations as trf

def poseQuat(pose,quat):
    pose.orientation.w = quat[0]
    pose.orientation.x = quat[1]
    pose.orientation.y = quat[2]
    pose.orientation.z = quat[3]

def QuatToList( quat):
    return [ quat.w,quat.x,quat.y,quat.z ]

def rotateQuat( pose, angle, which="roll"):
    quat = pose.orientation
    qlist = [ quat.w,quat.x,quat.y,quat.z ];
    elist = trf.euler_from_quaternion(qlist);
    newQuat = trf.quaternion_from_euler(ak=0,aj=0,ai=(elist[0]+angle));
    poseQuat(pose,newQuat);

def fillPose( poseStamped, pos, orient, header ):
    poseStamped.header = header;
    poseStamped.pose.position.x = pos[0];
    poseStamped.pose.position.y = pos[1];
    poseStamped.pose.position.z = pos[2];
    poseStamped.pose.orientation.w = orient[3];
    poseStamped.pose.orientation.x = orient[0];
    poseStamped.pose.orientation.y = orient[1];
    poseStamped.pose.orientation.z = orient[2];
