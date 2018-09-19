#! /usr/bin/env python

import rospy
import numpy as np
import actionlib
import tf

from std_msgs.msg import Header
from smarc_msgs.msg import EmptyAction
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class SurfaceAUV(object):
    
    def __init__(self, name):
        self.action_name = name

        # Get ROS params        
        self.auv_name = rospy.get_param(rospy.get_name() + '/auv_instance', 'lolo_auv')
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
        fin_angle = -60.
        thrust_level = 400.

        while not rospy.is_shutdown() and not self.end_condition():

            fin0angle = fin_angle # top
            fin1angle = fin_angle # left
            fin2angle = -fin_angle # down
            fin3angle = -fin_angle # right
            fin4angle = fin_angle # down
            fin5angle = fin_angle # right
            backfinangle = -0.1 # right

            self.thruster0.publish(header, thrust_level)
            self.thruster1.publish(header, thrust_level)

            self.fin0.publish(header, fin0angle)
            self.fin1.publish(header, fin1angle)
            self.fin2.publish(header, fin2angle)
            self.fin3.publish(header, fin3angle)
            self.fin4.publish(header, fin4angle)
            self.fin5.publish(header, fin5angle)
            self.backfin.publish(header, backfinangle)
    
            r.sleep()


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