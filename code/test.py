#!/usr/bin/env python

import rospy
import random
import actionlib

from actionlib_msgs.msg import *                                
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PatrolNav():
    def __init__(self):
        
        rospy.init_node('patrol_nav_node', anonymous=False)


        rospy.on_shutdown(self.shutdown)


        self.rest_time     = rospy.get_param("~rest_time", 5)
        self.keep_patrol   = rospy.get_param("~keep_patrol",   False)
        self.random_patrol = rospy.get_param("~random_patrol", False)
        self.patrol_type   = rospy.get_param("~patrol_type", 0)
        self.patrol_loop   = rospy.get_param("~patrol_loop", 2)
        self.patrol_time   = rospy.get_param("~patrol_time", 5)
        #z: -0.996269223692
    #   w: 0.0862996751122

        self.locations = dict()  
        self.locations['one']   = Pose(Point(3.48478,  1.91937, 0.000), Quaternion(0.000, 0.000,1, 0.00))# check
        self.locations['two']   = Pose(Point(0.4459,  2.6991, 0.000), Quaternion(0.000, 0.000, -1, 0.0536))# check
        self.locations['three'] = Pose(Point(2.1629,  2.5802, 0.000), Quaternion(0.000, 0.000, -0.05, 0.98))# check
        self.locations['four']  = Pose(Point(0.3174, -1.44786, 0.000), Quaternion(0.000, 0.000, 0.9999, 0.00001))# check
        self.locations['five']  = Pose(Point(0.29322, -0.58135, 0.000), Quaternion(0.000, 0.000, 0.016, 0.99974))# check
        self.locations['six']   = Pose(Point(4.3621, -1.4576, 0.000), Quaternion(0.000, 0.000, -0.3147, 0.949))# check

        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
                       'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")

        n_successes  = 0
        target_num   = 0
        location   = ""
        start_time = rospy.Time.now()
        locations_cnt = len(self.locations)
        # sequeue = ['six','five']
        sequeue = ['one', 'two', 'three', 'four', 'five', 'six']
        # sequeue = [ 'two', 'three', 'four', 'five', 'six']

         
        rospy.loginfo("Starting position navigation ")
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():

            #target_num = random.randint(0, locations_cnt-1)

            # Get the next location in the current sequence
            location = sequeue[target_num]

            # rospy.loginfo("target_num_value:"+str(target_num)) 

            rospy.loginfo("Going to: " + str(location))
            self.send_goal(location)

           

            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))


            # from sound_play.msg import SoundRequest
            # from sound_play.libsoundplay import SoundClient
            # soundhandle = SoundClient()
            



            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logerr("ERROR:Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                # soundhandle.play(1.0, 2.0)
                rospy.loginfo("Stop...")
                rospy.sleep(1.5)
                

            target_num += 1
            if target_num > 5:
               rospy.loginfo("Final stop...")
            

    def send_goal(self, locate):
        self.goal = MoveBaseGoal() 
        self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal) #send goal to move_base


    def shutdown(self):
        rospy.logwarn("Stopping the patrol...")

if __name__ == '__main__':
    try:
        PatrolNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")

