#!/usr/bin/env python

import rospy
from actionlib_tutorials.msg import *
import actionlib
from GUI import GUI
from threading import Thread


if __name__ == '__main__':

    rospy.init_node("countclientpy")
    end = rospy.get_param("~goal",0)
    client = actionlib.SimpleActionClient("countdown", countdownAction)
    client.wait_for_server();
    gui = GUI();
    goallist = Goal_List();

    def cancel():
        client.cancel_goal();

    gui.addButton("Cancel",cancel)

    data = [i*10 for i in range(5)]
    goallist.setList(data)
    goallist.printList()


    current = 0;
    target  = 0;
    final = len(data)

    goal = countdownGoal();
    running = True

    def doneCB(res,status):
        #res = client.get_goal_status_text()
        #num = client.get_state()

        print res ,",", status, ",",client.get_goal_status_text()

    while not rospy.is_shutdown():
        #gui.update();

        while running:
            if target < final:
                print "Current: %d, target: %d, final%d"%(current,target+1,final)
                goal.end = data[target];
                print "Sending..."
                client.send_goal(goal,done_cb=doneCB)
                intime = client.wait_for_result(timeout=rospy.Duration(3.0));
                if not intime:
                    print "Cancelled"
                    client.cancel_goal();
                    rospy.Rate(0.5).sleep()
                current+=1;
                target+=1;
            else:
                running = False;
                print "FINISHED!"

    #while not rospy.is_shutdown():
