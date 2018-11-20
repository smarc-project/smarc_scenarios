#! /usr/bin/env python

# Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import numpy as np
import actionlib
import tf

from std_msgs.msg import Header
from smarc_msgs.msg import EmptyAction, EmptyFeedback, EmptyResult
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class SurfaceAUV(object):
    
    def __init__(self, name):
        self.action_name = name

        # Get ROS params        
        self.auv_name = rospy.get_param('~auv_instance', 'lolo_auv')
        self.base_frame = rospy.get_param('~base_frame', self.auv_name + "/base_link")
        self.action_name = rospy.get_param('~surface_act_name', "/surface_auv_act")
        self.listener = tf.TransformListener()

        self.thruster0 = rospy.Publisher(self.auv_name + '/thrusters/0/input', FloatStamped, queue_size=10)
        self.thruster1 = rospy.Publisher(self.auv_name + '/thrusters/1/input', FloatStamped, queue_size=10)
        self.fin0 = rospy.Publisher(self.auv_name + '/fins/0/input', FloatStamped, queue_size=10)
        self.fin1 = rospy.Publisher(self.auv_name + '/fins/1/input', FloatStamped, queue_size=10)
        self.fin2 = rospy.Publisher(self.auv_name + '/fins/2/input', FloatStamped, queue_size=10)
        self.fin3 = rospy.Publisher(self.auv_name + '/fins/3/input', FloatStamped, queue_size=10)
        self.fin4 = rospy.Publisher(self.auv_name + '/fins/4/input', FloatStamped, queue_size=10)
        self.fin5 = rospy.Publisher(self.auv_name + '/fins/5/input', FloatStamped, queue_size=10)
        self.backfin = rospy.Publisher(self.auv_name + '/back_fins/0/input', FloatStamped, queue_size=10)

        self.act_server = actionlib.SimpleActionServer(self.action_name, EmptyAction, execute_cb=self.surface_auv_cb, auto_start = False)
        self.act_server.start()
      
    def surface_auv_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
                
        header = Header()
        fin_angle = -4.
        thrust_level = 200.

        feedback = EmptyFeedback()
        result = EmptyResult()

        success = True
        while not rospy.is_shutdown() and not self.end_condition():

            fin0angle = fin_angle # top
            fin1angle = fin_angle # left
            fin2angle = -fin_angle # down
            fin3angle = -fin_angle # right
            fin4angle = -fin_angle # down
            fin5angle = fin_angle # right
            backfinangle = -4. # right

            self.thruster0.publish(header, thrust_level)
            self.thruster1.publish(header, thrust_level)

            self.fin0.publish(header, fin0angle)
            self.fin1.publish(header, fin1angle)
            self.fin2.publish(header, fin2angle)
            self.fin3.publish(header, fin3angle)
            self.fin4.publish(header, fin4angle)
            self.fin5.publish(header, fin5angle)
            self.backfin.publish(header, backfinangle)
            
            # Send the feedback
            self.act_server.publish_feedback(feedback)
            
            # If the client preempts the action
            if self.act_server.is_preempt_requested():
                self.act_server.set_preempted()
                success = False
                break

            r.sleep()

        if success:
            self.act_server.set_succeeded(result)

        # Set to zero when finishing surface
        self.thruster0.publish(header, 0)
        self.thruster1.publish(header, 0)

        self.fin0.publish(header, 0)
        self.fin1.publish(header, 0)
        self.fin2.publish(header, 0)
        self.fin3.publish(header, 0)
        self.fin4.publish(header, 0)
        self.fin5.publish(header, 0)
        self.backfin.publish(header, 0)


    def end_condition(self):

        try:
            (trans, rot) = self.listener.lookupTransform("/world", self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

        # Check distance to navigation goal
        current_pos = np.array(trans)
        if current_pos[2] > -2 :
            rospy.loginfo("Reached surface!")
            return True
            
        return False

       
if __name__ == '__main__':
    rospy.init_node('surface_auv')
    server = SurfaceAUV(rospy.get_name())
    rospy.spin()    
