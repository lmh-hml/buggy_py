#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from PoseAndPath import PoseAndPath, EmptyPoseAndPathError
from Target_Pose import TargetPoser, GoalState
import Tkinter as tk
import threading
from enum import Enum
import exceptions
import sys

#Publishes target poses to move_base for navigation

class NavMode(Enum):
    IDLE=0
    RECORDING=1
    NAVIGATING=2

class App(tk.Tk):

    states = [ "Record" ,"Stop",
               "Navigate","Skip","Cancel","Cancel All",
               "Clear","Pop","Report"
             ];

    class LogMode(Enum):
        NORMAL="normal"
        ERROR ="error"
        SUCCESS="success"
        PROCESSING="processing"
        WARN="warn"

    class stdoutRedirect():
        def __init__(self, text_output):
            self.text_output = text_output;
        def write(self, txt):
            self.text_output.log_mode(txt, App.LogMode.NORMAL,"std");

    class stderrRedirect():
        def __init__(self, text_output):
            self.text_output = text_output;
        def write(self, txt):
            self.text_output.log_mode(txt, App.LogMode.ERROR,"std")

    def __init__(self, master=None):
        tk.Tk.__init__(self,master);
        self.frame = tk.Frame(self);
        self.buttons = {};
        self.texts = {};

        self.status_text = tk.Text(self,height=20,width = 50, bg="black",fg="grey");
        self.status_text.pack(side=tk.RIGHT, fill=tk.BOTH);
        self.texts["status_text"] = self.status_text;

        self.text = tk.Text(self,height=20,width = 50, bg="black",fg="white");
        self.text.pack(side=tk.RIGHT, fill=tk.BOTH);
        self.texts["text"] = self.text;

        self.frame.pack(side=tk.LEFT,fill=tk.Y);
        self.scroll = tk.Scrollbar(self);
        self.scroll.pack(side=tk.RIGHT, fill=tk.Y);
        self.mode = NavMode.IDLE;
        self.protocol("WM_DELETE_WINDOW",self.quit);
        self.text.tag_configure("normal",foreground="white");
        self.text.tag_configure("error",foreground="red");
        self.text.tag_configure("success",foreground="green");
        self.text.tag_configure("processing",foreground="yellow");
        self.text.tag_configure("warn",foreground="orange");
        #sys.stdout = self.stdoutRedirect(self);
        #sys.stderr = self.stderrRedirect(self);

    def addButton(self, name, callback=None):
        button = tk.Button(self.frame,text=name, command=callback);
        button.grid();
        self.buttons[name] = button;

    def printButtons(self):
        app.log( self.buttons);

    def log_mode(self, text, logMode,src="App", which="text"):
        output = self.texts[which];
        if not isinstance(text, basestring):
            to_write = src+" : " + str(text)+"\n";
            output.insert(tk.END, to_write,logMode.value);
        else:
            output.insert(tk.END, src+" : " +text+'\n',logMode.value);
        output.see(tk.END);

    def log(self, text):
        self.log_mode(text,self.LogMode.NORMAL);

    def error(self, text ):
        self.log_mode(text,self.LogMode.ERROR);

    def success(self, text ):
        self.log_mode(text,self.LogMode.SUCCESS);

    def progress(self, text ):
        self.log_mode(text,self.LogMode.PROCESSING);

    def warn(self, text ):
        self.log_mode(text,self.LogMode.WARN);

    def setMode(self, mode):
        self.mode = mode;
        if mode == NavMode.IDLE:
            for button in self.buttons:
                self.buttons[button].config(state="normal");
            self.buttons["Skip"].config(state="disabled");
            self.buttons["Cancel"].config(state="disabled");
            self.buttons["Cancel All"].config(state="disabled");

        elif mode == NavMode.RECORDING:
            self.buttons["Navigate"].config( state="disabled" );
            self.buttons["Record"].config(state="disabled");
            self.buttons["Cancel"].config(state="disabled");
            self.buttons["Cancel All"].config(state="disabled");
            self.buttons["Skip"].config(state="disabled");

        elif mode == NavMode.NAVIGATING:
            self.buttons["Navigate"].config(state="disabled");
            self.buttons["Record"].config(state="disabled");
            self.buttons["Stop"].config(state="disabled");
            self.buttons["Clear"].config(state="disabled");
            self.buttons["Pop"].config(state="disabled");
            self.buttons["Skip"].config(state="normal");
            self.buttons["Cancel"].config(state="normal");
            self.buttons["Cancel All"].config(state="normal");

    def quit(self):
        rospy.signal_shutdown("App closed");
        sys.stdout= sys.__stdout__;
        sys.stderr= sys.__stderr__;



class PathFollower:

    def __init__(self):
        self.tp = TargetPoser("move_base");
        self.pap = PoseAndPath("waypoint","map","base_link");
        self.state = GoalState(num=-1,msg="");
        self.mode = NavMode.IDLE;
        self.target = 0;
        self.current = -1;
        self.topic_name = "";

    def record_path(self,target_pose_topic):
        rospy.loginfo( "Subscribing to '%s'"%( target_pose_topic ) );
        self.sub = rospy.Subscriber(target_pose_topic,PoseStamped,pathfollower.pap.poseCB);
        self.topic_name = target_pose_topic;

    def stop_recording(self):
        if self.topic_name != "":
            rospy.loginfo("Unregistering from '%s'"%self.topic_name)
            self.sub.unregister();

    def report_path(self):
        path_size = self.pap.getSize();
        return path_size;

    def update_state(self):
        self.state = self.tp.getState();

    def report_state(self):
        text = "Status: %d"%self.state.num;
        if self.state.msg !="":
            text += "Msg: %s"%self.state.msg;
        return text;


    def follow_path_thread(self):
        try:
            rospy.loginfo("Going to Goal %d"%(self.target));
            pose = self.pap.getPoseStamped(self.target);
            self.tp.send_pose_goal(pose);
            self.tp.movebase.wait_for_result();
            self.report_state();
            print "State: %d,  %s"%(self.state.num,self.state.msg);
        except Exception as e:
            print e.message;
        print "FINISHING"

    def follow_path(self):
        size = self.report_path();
        if size <= 0:
            raise EmptyPoseAndPathError(self.pap.name);
        else:
            print "Thread!"
            try:
                t= threading.Thread(target=self.follow_path_thread);
                t.start();
            except Exception as e:
                self.error(e.message);

    def clearPath(self):
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

    app = App();
    rospy.init_node("waypoint");
    pathfollower = PathFollower();
    target_pose_topic = rospy.get_param("~target_pose","target_pose");

    def rec():
        app.log("RECORDING");
        app.setMode(NavMode.RECORDING)
        pathfollower.mode = NavMode.RECORDING;
        pathfollower.record_path("target_pose");

    def stop():
        app.warn("STOP RECORDING!")
        app.setMode(NavMode.IDLE);
        pathfollower.mode = NavMode.IDLE;
        pathfollower.stop_recording();

    def cancel():         #cancel current goal, and keep current goal the same
        app.warn("CANCEL!")
        app.setMode(NavMode.IDLE);
        pathfollower.tp.cancel_goal();

    def skip():           #cancel current goal, but forward current goal by one
        app.warn("SKIP!")
        app.setMode(NavMode.IDLE);
        pathfollower.tp.cancel_goal();
        pathfollower.target+=1;

    def nav():
        app.log("NAVIGATING");
        app.setMode(NavMode.NAVIGATING);
        try:
            pathfollower.follow_path();
        except EmptyPoseAndPathError as e:
            app.error(e.message);
            rospy.logerr(e.message)

    def reset():
        app.warn("CANCEL ALL GOALS")
        app.setMode(NavMode.IDLE);
        pathfollower.tp.cancel_goal(all_goals=True);

    def report():
        app.warn("REPORTING: ")
        app.log("Path size: %d"%(pathfollower.report_path()) );
        pathfollower.report_state();
        app.log("Target:%d"%(pathfollower.target));

    def clearPath():
        app.warn("CLEARING PATH!")
        try:
            pathfollower.clearPath();
        except EmptyPoseAndPathError as e:
            rospy.logerr(e.message);
            app.error(e.message);

    def popGoal():
        app.warn("REMOVE LAST GOAL!")
        try:
            pathfollower.remove_goal();
        except EmptyPoseAndPathError as e:
            rospy.logerr(e.message);
            app.error(e.message);

    states = [ ("Record",rec) , ("Stop", stop),("Navigate",nav),("Skip",skip),("Cancel",cancel),("Cancel All",reset),("Clear",clearPath),("Pop",popGoal),("Report",report) ];
    for i in range(len(states)):
        app.addButton(states[i][0], states[i][1]);
    app.setMode(NavMode.IDLE);

    last = rospy.Time.now();
    while not rospy.is_shutdown():
        now = rospy.Time.now();
        elapsed = now-last;
        if pathfollower.mode == NavMode.RECORDING:
            pathfollower.pap.publish();

        if elapsed.to_sec() >= 1.0:
            app.log_mode("Path size: %d"%(pathfollower.report_path()), logMode=app.LogMode.WARN,which="status_text" );
            app.log_mode(pathfollower.report_state(),logMode=app.LogMode.WARN,which="status_text" );
            app.log_mode("Target:%d"%(pathfollower.target), logMode=app.LogMode.WARN,which="status_text" );
            app.log_mode("", src="",logMode=app.LogMode.WARN,which="status_text" );
            last = now;

        app.update();

    print "Closing...";
    app.destroy();
