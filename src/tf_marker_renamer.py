#!/usr/bin/env python
'''
Script to localize a webcam with a kinect based on the tf's from ar localization.

This script is a messy, but necessary workaround to get the tf from the webcam to the kinect based
on the ar localization:

Due to running both the kinect and the webcam's alvar ar nodes, the tf tree gets confused since the tf_frame 
  is named alvar_marker_1. This breaks the tf tree, since alvar_marker_1 is given 2 parents (one the kinect and the
  other the webcamera.).  To avoid this a third, "clean tree", is created by listening to the ar_pose_marker topic
   for each camera type. There is then link from the renamed_webcam, the renamed_ar_marker_1, and the renamed_camera_depth_frame.

This tree is then used to correct the original kinect tree. First, based off the tf from the kinect to ar_marker_1, a new link
is added to the tree using the same tf, but with the frame id faux_ar_marker_1 so that it wont blip on and off.
 A tf lookup is done from the renamed_ar_marker_1 to renamed_webcam.
A new link in the tree is added to the tree, called unrotated_copied_web_camera using this tf. This is then rotated accordingly.




Author: Kory Kraft 
Date: 8-28-15

Copyright (c) 2015, Kory Kraft
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL KORY KRAFT BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import rospy

import tf

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_inverse

# msg types 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Header
from ar_track_alvar.msg import AlvarMarkers
from ar_track_alvar.msg import AlvarMarker

import numpy as np
import math 
from numpy.linalg import inv

class WorkAround():

    def __init__(self):
        
        self.webcam_ar_marker_1_sub = rospy.Subscriber('/webcam/ar_pose_marker', AlvarMarkers, self.webcam_marker_cb)#, queue_size=1)
        self.kinect_ar_marker_1_sub = rospy.Subscriber('/kinect/kinect_ar_pose_marker', AlvarMarkers, self.kinect_marker_cb)#, queue_size=1)

        self.br = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.tfros = tf.TransformerROS()

    def webcam_marker_cb(self, alvar_markers):
        # the tf given in the topic is from the webcamera to the ar marker

        ## do a check on the header to make sure its alvar_marker_1
        markers = alvar_markers.markers
        for marker in markers:
            if marker.id == 1:

                stamped_pose = marker.pose
                pose = stamped_pose.pose
               
                orig_trans = (pose.position.x, pose.position.y, pose.position.z)
                orig_rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                
                matrix_tf = self.tfros.fromTranslationRotation(orig_trans, orig_rot)

                np_inverted = np.linalg.inv(matrix_tf)

                inverted_trans = (np_inverted[0,3], np_inverted[1,3], np_inverted[2,3])
                inverted_rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, -pose.orientation.w)
                
                # for building the fake tree...
                self.br.sendTransform(inverted_trans,
                                      inverted_rot,
                                      rospy.Time.now(),
                                      "renamed_webcam",
                                      "renamed_ar_marker_1")

                # for adding on to the real tree ....
                self.br.sendTransform(inverted_trans,
                                      inverted_rot,
                                      rospy.Time.now(),
                                      "unrotated_copied_web_camera",
                                      "faux_ar_marker_1")

                 # ## Rotate the camera 90 to get it to line up correctly
                trans = (0.0, 0.0, 0.0)
                euler_x, euler_y, euler_z = (0.0, 0.0, 0.0)
                radian_2_add = 90 * (math.pi / 180)
                euler_z = euler_z + radian_2_add
                rot = quaternion_from_euler(euler_x, euler_y, euler_z)


                self.br.sendTransform(trans,
                                      rot, 
                                      rospy.Time.now(),
                                      "/copied_web_camera",
                                      "/unrotated_copied_web_camera")

                

                # self.br.sendTransform(orig_trans,
                #                       orig_rot,
                #                       rospy.Time.now(),
                #                       "faux_ar_marker_1_check",
                #                       "copied_web_camera")

            

    def kinect_marker_cb(self, alvar_markers):
        
        markers = alvar_markers.markers
        for marker in markers:
            if marker.id == 1:
                stamped_pose = marker.pose
                pose = stamped_pose.pose

                orig_trans = (pose.position.x, pose.position.y, pose.position.z)
                orig_rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

                negated_trans = (-pose.position.x, -pose.position.y, -pose.position.z)
                inverted_rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, -pose.orientation.w)

                ## for the clean tree....
                self.br.sendTransform(orig_trans,
                                       orig_rot,
                                       rospy.Time.now(),
                                       "renamed_ar_marker_1",
                                       "renamed_rgb_optical_frame") 

                ## add redundant ar_marker to the real tree (so it doesn't get lost)
                self.br.sendTransform(orig_trans,
                                       orig_rot,
                                       rospy.Time.now(),
                                       "faux_ar_marker_1",
                                       "camera_rgb_optical_frame")  


                # try:


                #     ### Transform for ar_marker_1 to webcam to add to the kinects original tree
                #     # Get the transform from the webcam to the renamed ar marker, and publish that
                #     (trans,rot) = self.tf_listener.lookupTransform("/renamed_ar_marker_1", "/renamed_webcam", rospy.Time(0))
                #     # print "tf from kinect to webcam"
                #     # print trans
                #     # print rot

                #     ## put the unrotated camera on the original kinect tree
                #     self.br.sendTransform(trans,
                #                           rot, 
                #                           rospy.Time.now(),
                #                           "/unrotated_copied_web_camera",
                #                           "/faux_ar_marker_1")
                    

                #     # ## Rotate the camera 90 to get it to line up correctly
                #     trans = (0.0, 0.0, 0.0)

                #     euler_x, euler_y, euler_z = (0.0, 0.0, 0.0)
                #     # radian_2_add = 110 * (math.pi / 180)
                #     # euler_z = euler_z + radian_2_add
                #     rot = quaternion_from_euler(euler_x, euler_y, euler_z)


                #     self.br.sendTransform(trans,
                #                           rot, 
                #                           rospy.Time.now(),
                #                           "/copied_web_camera",
                #                           "/unrotated_copied_web_camera")

                #     (trans,rot) = self.tf_listener.lookupTransform("/renamed_camera_depth_frame", "/renamed_webcam", rospy.Time(0))

                #     self.br.sendTransform(trans,
                #                           rot, 
                #                           rospy.Time.now(),
                #                           "/blimey_camera",
                #                           "/camera_depth_frame")
                   

                # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #     continue

            


def main():
    rospy.init_node('ar_tf_id_changer', anonymous=True)

    workAround = WorkAround()

    rospy.spin()

if __name__ == '__main__':
    main()