#!/usr/bin/env python
#from __future__ import print_function

import rospy
import sys
import cv2
import numpy as np
from datetime import datetime
#import tensorflow as tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, PoseStamped
import tf

from nav_msgs.msg import Path


def construct_message(frame_id, point1, point2=None):
    # create a Path message with whatever frame we received the localisation in
    msg = Path()
    # Path.poses is a PoseStamped list so we have to create these objects
    p1 = PoseStamped()
    p2 = PoseStamped()
    msg.header.frame_id = p1.header.frame_id = p2.header.frame_id = frame_id #self.world_frame
    msg.header.stamp = rospy.Time.now()

    if point1 is not None:

        point1 = np.array(point1)
        point2 = np.array(point2)

        point2 = point2 + 10.*(point2 - point1)
        point1 = point1 + 10.*(point1 - point2)

        p1.pose.position.x = point1[0]
        p1.pose.position.y = point1[1]
        p1.pose.position.z = point1[2]
        # p1.pose.orientation.w = 1.
        msg.poses.append(p1)

        p2.pose.position.x = point2[0]
        p2.pose.position.y = point2[1]
        p2.pose.position.z = point2[2]
        # p2.pose.orientation.w = 1.
        msg.poses.append(p2)

    return msg


class PipeNotFound(Warning):
    def __init__(self, frame_id, publisher):
        msg = construct_message(frame_id, None)
        publisher.publish(msg)
        print str(datetime.now()) + " Pipeline not found"


class VisualPipelineLocator:
    def __init__(self, image_topic, output_topic, visualize=False):

        self.subscriber = rospy.Subscriber(image_topic, Image, self.process_ros_image)

        self.publisher = rospy.Publisher(output_topic, Path, queue_size=10)
        self.crop_y = 150
        self.world_frame = "world"

        self.visualize = visualize
        self._frame_id = 'lolo_auv/camera_link_optical'

        self.bridge = CvBridge()
        self.tf = tf.TransformListener()

    def flatten(self, nested_list):
        flat_list = [item for sublist in nested_list for item in sublist]
        return flat_list

    def color_threshold(self, image):
        orange_min = np.array([10, 100, 20], np.uint8)
        orange_max = np.array([25, 255, 255], np.uint8)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        thresholded_image = cv2.inRange(hsv, orange_min, orange_max)
        return thresholded_image

    def compute_axis(self, lines):
        try:
            if lines is None:
                raise PipeNotFound(self._frame_id, self.publisher)
            else:
                lin = np.array(lines)
                axis = np.ndarray.tolist(np.mean(lin, axis=0))
                axis = self.flatten(axis)
                return axis
        except PipeNotFound:
            pass

    def calculate_2D_segment_ends(self, rho, theta, height, width):
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        xN = -b
        yN = a

        xheight = [x0 + (height - 1 - y0)/yN*xN, height - 1]
        ywidth = [width - 1, y0 + (width - 1 - x0)/xN*yN]
        xorigin = [x0 - y0 / yN * xN, 0]
        yorigin = [0, y0 - x0 / xN * yN]

        points = np.array([xheight, ywidth, xorigin, yorigin], dtype=int)
        #print points
        indices = \
        np.logical_and(np.logical_and(points[:, 0] < width, points[:, 0] >= 0),
                       np.logical_and(points[:, 1] < height, points[:, 1] >= 0))
        indices = np.nonzero(indices)[0]
        # print indices
        # print points
        points = points[indices, :].tolist()

        return tuple(points[0]), tuple(points[1])

        #try:
        #    m = (y1 - y0)/(x1 - x0)
        #except ZeroDivisionError:
        #    m = 9999
        #finally:

        #return end1, end2

    def print_lines(self, image, lines):
        #try:
        if lines is None:
            print "No lines found"
            raise PipeNotFound(self._frame_id, self.publisher)
        else:
            print "Lines: ", lines
            copy = image
            rho = lines[0]
            theta = lines[1]
            #x1, y1, x2, y2 = self.calculate_2D_segment_ends(rho, theta)
            p1, p2 = self.calculate_2D_segment_ends(rho, theta, image.shape[0], image.shape[1])
            cv2.circle(copy, p1, 10, (255, 255, 255), 3)
            cv2.circle(copy, p2, 10, (255, 255, 255), 3)

            p1 = (p1[0], p1[1]+self.crop_y)
            p2 = (p2[0], p2[1]+self.crop_y)

            #cv2.line(copy, (x1, y1), (x2, y2), (0, 0, 255), 2)
            return copy, p1+(1.,), p2+(1.,)
        # except TypeError as e:
        #     try:
        #         copy = image
        #         print "Got an exception:", e
        #         #p1, p2 = self.calculate_2D_segment_ends(rho, theta, image.shape[1], image.shape[2])
        #         #cv2.circle(copy, p1, 10, (255, 255, 255), 3)
        #         #cv2.circle(copy, p2, 10, (255, 255, 255), 3)
        #         #rho, theta = lines
        #         #x1, y1, x2, y2 = self.calculate_2D_segment_ends(rho, theta)
        #         #cv2.line(copy, (x1, y1), (x2, y2), (0, 0, 255), 2)
        #         return copy
        #     except:
        #         print "The problem is not that the line is single"
        # except:
        #     # print "There is another kind of error"
        #     return image

    def project_to_world(self, p1, p2):

        fx = 300.83
        fy = fx
        cx = 384.5
        cy = 246.5

        p1 = np.array([(p1[0]-cx)/fx, (p1[1]-cy)/fy, 1.])
        p2 = np.array([(p2[0] - cx) / fx, (p2[1] - cy) / fy, 1.])

        try:
            self.tf.waitForTransform(self.world_frame, self._frame_id, rospy.Time(0), rospy.Duration(4.0))
            (position, quaternion) = self.tf.lookupTransform(self.world_frame, self._frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Could not get transform between %s and %s, quitting...", self._frame_id, self.world_frame)
            sys.exit(-1)

        #if self.tf.frameExists(self._frame_id) and self.tf.frameExists("world"):
        #    t = self.tf.getLatestCommonTime(self._frame_id, "world")
        #    position, quaternion = self.tf.lookupTransform(self._frame_id, "world", t)
        #    print position, quaternion
        #else:
        #    print "We do not have world frame or ", self._frame_id

        euler = tf.transformations.euler_from_quaternion(quaternion)
        rot = tf.transformations.euler_matrix(euler[0], euler[1], euler[2])[:3, :3]

        seafloor_height = -95.
        print np.dot(rot[2,:], p1)
        print position
        alpha1 = (seafloor_height-position[2])/np.dot(rot[2,:], p1)
        alpha2 = (seafloor_height - position[2]) / np.dot(rot[2, :], p2)

        p1 = alpha1*np.matmul(rot, p1) + position
        p2 = alpha2 * np.matmul(rot, p2) + position

        print p1, p2

        return tuple(p1.tolist()), tuple(p2.tolist())

    def process_cv2_image(self, input_image):
        cropped_image = input_image[self.crop_y:]  # crop submarine shell
        blur = cv2.GaussianBlur(cropped_image, (0, 0), 10)
        thresholded = self.color_threshold(blur)
        edges = cv2.Canny(thresholded, 100, 200)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
        pipe_axis = self.compute_axis(lines)
        visualize, p1, p2 = self.print_lines(cropped_image, pipe_axis)
        p1, p2 = self.project_to_world(p1, p2)
        if self.visualize:
            cv2.imshow("Image window", visualize)
            cv2.waitKey(3)
        return pipe_axis, p1, p2  # as (rho, theta)

    def process_ros_image(self, image_message):
        #try:
        cv_image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        pipe_axis, p1, p2 = self.process_cv2_image(cv_image)
        #modify from here
        msg = construct_message(self.world_frame, p1, p2)
        #msg.linear.x, msg.linear.y = pipe_axis
        self.publisher.publish(msg)
        # except TypeError:
        #     pass
        # except CvBridgeError as e:
        #     print(e)


def main(args):
    rospy.init_node('visual_pipeline_locator', anonymous=True)
    locator = VisualPipelineLocator(image_topic='/lolo_auv/lolo_auv/camera/camera_image',
                                    output_topic='/lolo_auv/lolo_auv/camera/pipeline_locator',
                                    visualize=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down pipe locator")
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
