#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from collections import namedtuple;

GoalState = namedtuple("GoalState",'num, msg');

class TargetPoser:

    def __init__(self,server_name):
        self.movebase = actionlib.SimpleActionClient(server_name,MoveBaseAction);
        self.poseStamped = PoseStamped();
        self.state   = GoalState(num=-1 ,msg="");
        self.wait_for_server();

    def wait_for_server(self):
        print "Wait for server..."
        connected = self.movebase.wait_for_server(timeout=rospy.Duration(1.0));
        if connected == True:
            rospy.loginfo("Server connected!");
        else:
            rospy.logerr("Unable to connect to server")

    def getState(self):
        """Gets the status and text for the current goal.
           Returns a named tuple: GoalState( num, msg)
           num: the goal status integer
           msg: the goal status text. Can be empty, useful for aborts.
        """
        state_num = self.movebase.get_state();
        state_msg = self.movebase.get_goal_status_text();
        goal_state = GoalState(num = state_num, msg=state_msg)
        return goal_state;

    def feedbackCB(self,fb):
        pass

    def doneCB(self,state,res):
        self.state = GoalState(num=state,msg=self.movebase.get_goal_status_text());
        res = self.movebase.get_result();
        print "Goal Callback!! state: %d,  %s"%(self.state.num,self.state.msg);
        print str(res);

    def send_pose_goal(self,poseStamped,whenDone=None,whenActive=None,whenFeedback=None):
        #print "sending goal: %s"%(str(poseStamped));
        if whenDone == None:
            whenDone = self.doneCB
        if whenFeedback == None:
            whenFeedback = self.feedbackCB;
            
        mbgoal = MoveBaseActionGoal();
        mbgoal.goal.target_pose = poseStamped;
        self.movebase.send_goal(mbgoal.goal, done_cb=whenDone,active_cb=whenActive,feedback_cb=whenFeedback);

    def cancel_goal(self, all_goals=False):
        if not all_goals:
            self.movebase.cancel_goal();
        else:
            self.movebase.cancel_all_goals();

    # for use as a callback
    def send_poseCB(self):
        send_pose_goal(self.poseStamped);

    def getPose( self, poseStamped ):
        self.poseStamped = poseStamped;
#CLASS
