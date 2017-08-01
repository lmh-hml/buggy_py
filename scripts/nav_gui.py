#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from PoseAndPath import PoseAndPath, EmptyPoseAndPathError
from Target_Pose import TargetPoser, GoalState
from GUI import GUI, PrintQueue,tk
from Queue import Queue
import threading
from enum import Enum
import exceptions
import sys

#Publishes target poses to move_base for navigation

class NavMode(Enum):
    IDLE=0
    RECORDING=1
    NAVIGATING=2


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

    def report_path_size(self):
        path_size = self.pap.getSize();
        return path_size;

    def update_state(self):
        self.state = self.tp.getState();

    def report_state(self):
        """returns current actionlib client state as a string that indicates state id and msg, if any."""
        text = "Status: %d"%self.state.num;
        if self.state.msg !="":
            text += ", Msg: %s"%self.state.msg;
        return text;

    def follow_path_thread(self,size):
        running = True;
        try:
            while running:
                rospy.loginfo("Going to Goal %d of %d"%(self.target,size));
                pose = self.pap.getPoseStamped(self.target);
                self.tp.send_pose_goal(pose);
                self.tp.movebase.wait_for_result();
                self.update_state();
                textQ.put("State: %d, msg: %s"%(self.state.num,self.state.msg));

                if self.state.num != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                    running = False;
                else:
                    self.target+=1;
                    if self.target>=size:
                        running=False;
                        textQ.put("Path finished!!!!")
                        self.target = 0;

        except Exception as e:
            print e.message;
        textQ.put("THREAD EXITS");

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
    if mode == NavMode.IDLE:
        for button in gui.buttons:
            gui.buttons[button].config(state="normal");
        gui.buttons["Skip"].config(state="disabled");
        gui.buttons["Cancel"].config(state="disabled");
        gui.buttons["Cancel All"].config(state="disabled");

    elif mode == NavMode.RECORDING:
        gui.buttons["Navigate"].config( state="disabled" );
        gui.buttons["Record"].config(state="disabled");
        gui.buttons["Cancel"].config(state="disabled");
        gui.buttons["Cancel All"].config(state="disabled");
        gui.buttons["Skip"].config(state="disabled");

    elif mode == NavMode.NAVIGATING:
        gui.buttons["Navigate"].config(state="disabled");
        gui.buttons["Record"].config(state="disabled");
        gui.buttons["Stop Rec"].config(state="disabled");
        gui.buttons["Clear"].config(state="disabled");
        gui.buttons["Pop"].config(state="disabled");
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

    sys.stdout = PrintQueue(infoQ);
    sys.stderr = PrintQueue(errorQ);

    app.addText("State",70,20, side = tk.BOTTOM);
    app.setOutput("State")

    intro = """
INSTRUCTIONS:
Record: Gui subsribes to topic to record waypoints for path
Stop Rec  : Stop recording.
Navigate: begin auto-navigation
Cancel: cancel current goal
Skip:   cancel current goal and increase target by 1
Cancel all: cancel all goals and reset target to 0
Pop: removes latest waypoints
Clear: removes all waypoints
Report: Prints actionlib goal status, number of waypoints and current target, etc.
    """
    textQ.put(intro);

    def rec():
        textQ.put("RECORDING")
        setMode(app, pathfollower,NavMode.RECORDING);
        pathfollower.mode = NavMode.RECORDING;
        pathfollower.record_path(target_pose_topic);
        pass;

    def stop():
        textQ.put("STOP")
        setMode(app,pathfollower,NavMode.IDLE);
        pathfollower.mode = NavMode.IDLE;
        pathfollower.stop_recording();
        pass

    def cancel():
        textQ.put("CANCEL CURRENT GOAL")
        setMode(app,pathfollower,NavMode.IDLE);
        pathfollower.tp.cancel_goal();
        pass

    def skip():           #cancel current goal, but forward current goal by one
        textQ.put("SKIP CURRENT GOAL")
        setMode(app,pathfollower,NavMode.IDLE);
        pass;

    def nav():
        textQ.put("NAVIGATING")
        setMode(app,pathfollower,NavMode.NAVIGATING);
        try:
            pathfollower.follow_path();
        except EmptyPoseAndPathError as e:
            rospy.logerr(e.message)
        pass

    def reset():
        textQ.put("RESET")
        setMode(app,pathfollower,NavMode.IDLE);
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
        stateQ.put("Target:%d"%(pathfollower.target));
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


    states = [ ("Record",rec) , ("Stop Rec", stop),("Navigate",nav),("Skip",skip),("Cancel",cancel),("Cancel All",reset),("Clear",clearPath),("Pop",popGoal),("Report",report) ];

    for i in range(len(states)):
        app.addButton(states[i][0], states[i][1]);

    last = rospy.Time.now();
    while not rospy.is_shutdown():

        if app.running:
            app.update();

            if pathfollower.mode == NavMode.RECORDING:
                pathfollower.pap.publish();

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