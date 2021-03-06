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
from load_path import load_poses_from_file

PI = 3.14159265
PIOVERONEEIGHTY = PI/180.0

def rotateQuat( pose, angle, which="roll"):
    quat = pose.orientation;
    qlist = [ quat.w,quat.x,quat.y,quat.z ];
    elist = euler_from_quaternion(qlist);
    newQuat = quaternion_from_euler(ak=0,aj=0,ai=(elist[0]+angle));
    pose.orientation.w = newQuat[0]
    pose.orientation.x = newQuat[1]
    pose.orientation.y = newQuat[2]
    pose.orientation.z = newQuat[3]

IDLE=0
RECORDING=1
NAVIGATING=2

class PathFollower:

    def __init__(self):
        self.tp = TargetPoser("move_base");
        self.pap = PoseAndPath("waypoint","map","base_link");
        self.state = GoalState(num=-1,msg="");
        self.mode = IDLE;
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
            rospy.loginfo("Unregistering from '%s'"%self.topic_name)
            self.sub.unregister();

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
                        print("Path finished!!!!")
                        self.target = 0;

        except Exception as e:
            print e.message;
        textQ.put("THREAD EXITS WITH STATUS: %d"%self.state.num);

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


def setMode(gui, pathfollower, mode):
    pathfollower.mode = mode;
    if mode == IDLE:
        for button in gui.buttons:
            gui.buttons[button].config(state="normal");
        gui.buttons["Skip"].config(state="disabled");
        gui.buttons["Cancel"].config(state="disabled");
        gui.buttons["Cancel All"].config(state="disabled");

    elif mode == RECORDING:
        gui.buttons["Navigate"].config( state="disabled" );
        gui.buttons["Record"].config(state="disabled");
        gui.buttons["Cancel"].config(state="disabled");
        gui.buttons["Cancel All"].config(state="disabled");
        gui.buttons["Skip"].config(state="disabled");

    elif mode == NAVIGATING:
        gui.buttons["Navigate"].config(state="disabled");
        gui.buttons["Record"].config(state="disabled");
        gui.buttons["Stop Rec"].config(state="disabled");
        gui.buttons["Clear"].config(state="disabled");
        gui.buttons["Pop"].config(state="disabled");
        gui.buttons["Flip"].config(state="disabled");
        gui.buttons["Skip"].config(state="normal");
        gui.buttons["Cancel"].config(state="normal");
        gui.buttons["Cancel All"].config(state="normal");

textQ  = Queue(); #for app printing
stateQ = Queue(); #for state reporting
infoQ = Queue();  #for ros log info
errorQ = Queue(); #for ros log error

if __name__ == '__main__':

    app =GUI();
    rospy.init_node("nav_gui");
    pathfollower = PathFollower();
    target_pose_topic = rospy.get_param("~target_pose","target_pose");
    visualize = rospy.get_param("~visual",True);
    path_dir = rospy.get_param("~save_load_dir","/home/buggy/catkin_ws/src/buggy_py/txt/")
    map_name = rospy.get_param("map_name","");

    #Override print and logs to the designated queues. Terminal will not show anything from this node beyond this point.
    #COMMENT THESE TWO OUT WHEN DEBUGGING CRASHES OR ERRORS TO SHOW OUTPUT IN TERMINAL
    sys.stdout = PrintQueue(infoQ);
    sys.stderr = PrintQueue(errorQ);

    begin_pub = rospy.Publisher("Begin_pose",PoseStamped,queue_size=100);

    app.addText("State",70,20);
    app.setOutput("State")

    def rec():
        textQ.put("RECORDING")
        setMode(app, pathfollower,RECORDING);
        pathfollower.mode = RECORDING;
        pathfollower.record_path(target_pose_topic);
        pass;

    def stop():
        textQ.put("STOP")
        setMode(app,pathfollower,IDLE);
        pathfollower.mode = IDLE;
        pathfollower.stop_recording();
        pass

    def cancel():
        textQ.put("CANCEL CURRENT GOAL")
        setMode(app,pathfollower,IDLE);
        pathfollower.tp.cancel_goal();
        pass

    def skip():           #cancel current goal, but forward current goal by one
        textQ.put("SKIP CURRENT GOAL")
        setMode(app,pathfollower,IDLE);
        pass;

    def nav():
        textQ.put("NAVIGATING")
        setMode(app,pathfollower,NAVIGATING);
        try:
            pathfollower.follow_path();
        except EmptyPoseAndPathError as e:
            rospy.logerr(e.message)
        pass

    def reset():
        textQ.put("RESET")
        setMode(app,pathfollower,IDLE);
        pathfollower.tp.cancel_goal(all_goals=True);
        self.target = 0;
        pass

    def report():
        textQ.put("Report");
        pathfollower.update_state()
        s = pathfollower.report_state();
        n = pathfollower.report_path_size();
        stateQ.put(s);
        stateQ.put("Path size: %d"%n);
        stateQ.put("Target:%d, Starting:%d"%(pathfollower.target, pathfollower.begin));
        pass

    def clearPath():
        app.setOutput("State")
        app.warn("CLEARING PATH!")
        try:
            pathfollower.clearPath();
        except EmptyPoseAndPathError as e:
            app.error(e.message);
        pass

    def popGoal():
        app.setOutput("State")
        app.warn("REMOVE LAST GOAL!")
        try:
            pathfollower.remove_goal();
        except EmptyPoseAndPathError as e:
            app.error(e.message);
        pass

    def save_path():
        saved_file = tkfile.asksaveasfilename(initialdir=path_dir);
        if saved_file == "":
            return
        textQ.put(" SAVING PATH TO %s"%saved_file);
        pathfollower.pap.write_path_simple(saved_file,map=map_name);
        pass

    def load_file():
        load_file = tkfile.askopenfilename(initialdir=path_dir);
        if load_file == "":
            return
        textQ.put("Loading from file %s"%load_file);
        success = load_poses_from_file(load_file,pathfollower.pap);
        pathfollower.begin = 0;
        pathfollower.target = 0;
        pathfollower.step = 1;
        if not success:
            textQ.put("UNABLE TO LOAD FILE")
        pass

    def flipPath():
        size =  pathfollower.report_path_size();
        textQ.put("Flipping %d poses!"%size);
        if size<=0:
            return
        for i in range( size ):
            pose = pathfollower.pap.getPoseStamped(i);
            rotateQuat(pose.pose, PI)
        pathfollower.step=--pathfollower.step;
        if pathfollower.begin<=0:
            pathfollower.begin= pathfollower.report_path_size()-1;
        else:
            pathfollower.begin = 0;
        pathfollower.update_begin_pose();

    states = [ ("Record",rec) , ("Stop Rec", stop),
    ("Navigate",nav),("Skip",skip),("Cancel",cancel),("Cancel All",reset),
    ("Clear",clearPath),("Pop",popGoal), ("Flip",flipPath),
    ("Save",save_path),("Load", load_file),
    ("Report",report) ];

    for i in range(len(states)):
        app.addButton(states[i][0], states[i][1]);

    last = rospy.Time.now();
    while not rospy.is_shutdown():

        if app.running:
            app.update();

            if pathfollower.mode == RECORDING and visualize == True:
                pathfollower.pap.publish();
                begin_pub.publish( pathfollower.begin_end[0]);

            #print logs from queues onto gui textboxes
            app.logFromQueue(textQ,output="State",mode =GUI.LogMode.WARN);
            app.logFromQueue(stateQ,output="State", mode = GUI.LogMode.PROCESSING);
            app.logFromQueue(infoQ,output="Info",src="ROS");
            app.logFromQueue(errorQ,output="Info",mode=GUI.LogMode.ERROR,src="ROS");

        else:
            break;

    rospy.signal_shutdown("app closed");
    print "Closing...";
    app.destroy();
