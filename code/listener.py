#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import time

def callback(data):
    id = data.markers[0].id
    distance = data.markers[0].pose.pose.position.z
    rospy.loginfo('id=%d and distance=%f', id, distance)
    # Import here so that usage is fast.
    from sound_play.msg import SoundRequest
    from sound_play.libsoundplay import SoundClient
    # rospy.init_node('play', anonymous=True)
    soundhandle = SoundClient()
    if id !=255:
        rospy.sleep(1)
        for i in range(id):
            num =1
            volume = 2.0
            rospy.loginfo('Playing sound %i.' % num)

            soundhandle.play(num, volume)
            rospy.sleep(0.6)

        rospy.sleep(5)

    
    
    # rospy.sleep(5)
#    else


    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

    rospy.loginfo('done message')

    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback, queue_size=1)

    rospy.loginfo('done message')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() 

if __name__ == '__main__':
    listener()


# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.
# #
# # Revision $Id$

# ## Simple talker demo that listens to std_msgs/Strings published 
# ## to the 'chatter' topic

# import sys
# import rospy
# import os
# from ar_track_alvar_msgs.msg import AlvarMarkers
# from std_msgs.msg import String
# from sound_play.msg import SoundRequest
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Vector3
# from sound_play.libsoundplay import SoundClient

# def callback(data):
#     id = data.markers[0].id
#     distance = data.markers[0].pose.pose.position.z
#     rospy.loginfo('I heard id=%d and distance=%f', id, distance)

#     soundhandle = SoundClient()
#     rospy.sleep(1)
#     if id < 18 and id > 0:
#         rospy.loginfo('Playing sound %d.', id)
#         for k in range(id):
#             soundhandle.play(1, 1)
#             rospy.sleep(1)
    
#     if id == 0:
#         if distance < 0.60:
#             rospy.loginfo("Stopping the turtlebot")
#             os.system('rosnode kill detection')
#             os.system('rosnode kill Velocity')
#             vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
#             # rate = rospy.rate(1000)
#             stop = Twist()
#             stop.linear.x = 0
#             stop.linear.y = 0
#             stop.linear.z = 0
#             stop.angular.x = 0
#             stop.angular.y = 0
#             stop.angular.z = 0
#             while (True):
#                 vel_pub.publish(stop)
#             rospy.sleep(1)

#     rospy.sleep(1)

# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)

#     rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

#     rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback, queue_size=1)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

# if __name__ == '__main__':
#     listener()
