#!/usr/bin/python

from __future__ import division, print_function

import scipy.special
#import scipy.misc
#import scipy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from smarc_msgs.msg import EmptyAction, EmptyFeedback, EmptyResult
import rospy
import tf
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib
import math

import tf2_ros
import tf2_geometry_msgs

class PipeDetector(object):
    
    # create messages that are used to publish feedback/result
    _feedback = EmptyFeedback()
    _result = EmptyResult()

    def callback(self, fls_msg):

        if not self.is_executing:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(fls_msg, desired_encoding="passthrough") #"float32")
        except CvBridgeError as e:
            print(e)

        #print("Got pose!")
        #print(cv_image.shape)
        #print(cv_image)
        #img = cv2.imread('dave.jpg')
        #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        cv_image = 255. * cv_image / cv_image.max() #normalizes data in range 0 - 255
        img = cv_image.astype(np.uint8)

        #cv2.imshow("Image window", img)
        #cv2.waitKey(3)
        
        kernel = np.ones((9, 9), np.float32)/81.
        dst = cv2.filter2D(img, -1, kernel)
        #dst = cv2.filter2D(dst, -1, kernel)
        edges = cv2.Canny(dst, 30, 50, apertureSize=3)
        #cv2.ellipse(scan, center, full_axes, -90, -fov/2., fov/2., 0.2, -1);
        fov = 100.
        third_axes = (int(edges.shape[0]/2.8), int(edges.shape[0]/2.8))
        full_axes = (int(edges.shape[0]/1.05), int(edges.shape[0]/1.05))
        center = (int(edges.shape[1]/2), int(edges.shape[0]))
        # cv2.ellipse(img, center, axes, angle, startAngle, endAngle, color[, thickness[, lineType[, shift]]])
        cv2.ellipse(edges, center=center, axes=third_axes, angle=-90, startAngle=-fov/2., endAngle=fov/2., color=0, thickness=-1)
        mask = np.zeros(edges.shape, dtype=np.uint8)
        cv2.ellipse(mask, center=center, axes=full_axes, angle=-90, startAngle=-fov/2.5, endAngle=fov/2.5, color=255, thickness=-1)

        edges = cv2.bitwise_and(edges, edges, mask=mask)

        edges = np.flipud(edges.transpose()) # rotate -90 deg
        dst = np.flipud(dst.transpose()).copy() # rotate -90 deg
        #cv2.imshow("Edges", edges)
        #cv2.waitKey(3)
        #print(dst.shape)

        #lines = cv2.HoughLines(edges, 10, 0.1, 200, min_theta=0.75*math.pi, max_theta=1.*math.pi)
        lines = cv2.HoughLines(edges, 10, 0.02, 100, min_theta=0.25*math.pi, max_theta=0.75*math.pi)
        #print("Got ", len(lines[0]), " lines")
        #print(lines)
        if lines is None:
            return

        #print(lines[0])
        #for i in range(0, min(len(lines), 1)):
        (rho, theta) = lines[0][0]
        #print("Rho, theta: ", rho, theta)
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        y = int(y0 + x0/b*a)

        cv2.line(dst,(x1,y1),(x2,y2), 255, 3)
        cv2.circle(dst, (0, y), 10, 255)

        real_range = 17.
        mapped_range = float(edges.shape[1])
        scale = real_range/mapped_range
        point = scale*np.array([float(edges.shape[0]/2.-y), 0., float(edges.shape[1])])

        #cv2.imshow("Image window", dst)
        #cv2.waitKey(3)

        transform = self.tf_buffer.lookup_transform("world", # target frame
                                               "lolo_auv_1/forward_sonardown_optical_frame", #source frame
                                               rospy.Time(0), #get the tf at first available time
                                               rospy.Duration(1.0)) #wait for 1 second
        quaternion = tf.transformations.quaternion_from_euler(0., theta-math.pi, 0.)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "/lolo_auv_1/forward_sonardown_optical_frame"
        pose_stamped.header.stamp = rospy.get_rostime()
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.position.z = point[2]
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        
        quaternion = [pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        quaternion = tf.transformations.quaternion_from_euler(0., 0., euler[2])
        pose_transformed.pose.orientation.x = quaternion[0]
        pose_transformed.pose.orientation.y = quaternion[1]
        pose_transformed.pose.orientation.z = quaternion[2]
        pose_transformed.pose.orientation.w = quaternion[3]

        pose_transformed.pose.position.z = -85.

        self.goal_pub.publish(pose_transformed)

    def execute_cb(self, goal):

        #success = True
        self.is_executing = True
        rospy.loginfo("Running detection!")
        
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown() and not self._as.is_preempt_requested():
            self._as.publish_feedback(self._feedback)
            r.sleep()

        self.is_executing = False
        self.goal_pub.publish(PoseStamped())
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        
        #if success:
        #    #self._result.sequence = self._feedback.sequence
        #    rospy.loginfo('%s: Succeeded' % self._action_name)
        #    self._as.set_succeeded(self._result)

    def __init__(self, name):
        
        self.is_executing = False
        self._action_name = name
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        #self.image_pub = rospy.Publisher("/pipe_detection", Image)

        self.bridge = CvBridge()
        self.image_sub = None #rospy.Subscriber("/lolo_auv_1/depth/image_raw_raw_sonar", Image, self.callback)

        #rospy.Timer(rospy.Duration(1), self.timer_callback)

        self._as = actionlib.SimpleActionServer(self._action_name, EmptyAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self._action_name)

        #self.image_sub = rospy.Subscriber("/lolo_auv_1/depth/image_raw_raw_sonar", Image, self.callback)
        self.image_sub = rospy.Subscriber("/fls_input", Image, self.callback)

        rospy.spin()
        
        #r = rospy.Rate(0.1) # 10hz
        #while not rospy.is_shutdown():
        #   #pub.publish("hello world")
        #   if self.nav_goal is not None:
        #       path = self.plan()
        #       self.pub.publish(path)
        #   r.sleep()

if __name__ == '__main__':
    rospy.init_node('fls_pipe_detector')
    planner = PipeDetector(rospy.get_name())
