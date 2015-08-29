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
        
        self.webcam_ar_marker_1_sub = rospy.Subscriber('/webcam/ar_pose_marker', AlvarMarkers, self.webcam_marker_cb, queue_size=1)

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

                ############## Checks for tree building! #################3

                ###### Checks the direction from the ar marker to the camera, with ar marker as the parent #########

                self.br.sendTransform(orig_trans,
                                      orig_rot,
                                      rospy.Time.now(),
                                      "webcam_check1",
                                      "ar_marker_1")

                self.br.sendTransform(orig_trans,
                                      inverted_rot,
                                      rospy.Time.now(),
                                      "webcam_check2",
                                      "ar_marker_1")

                self.br.sendTransform(inverted_trans,
                                      orig_rot,
                                      rospy.Time.now(),
                                      "webcam_check3",
                                      "ar_marker_1")

                ### does the inverse as desired
                self.br.sendTransform(inverted_trans,
                                      inverted_rot,
                                      rospy.Time.now(),
                                      "webcam_check4",
                                      "ar_marker_1")

                ############### Checks the direction with the webcamera as the parent, and the ar tag as the child ##########33
                
                # Works as expected....the tf given in the topic is from the webcamera to the ar marker
                self.br.sendTransform(orig_trans,
                                      orig_rot,
                                      rospy.Time.now(),
                                      "ar_check1",
                                      "web_camera")   

                ## keeps sme orientation, but goes backward in space...
                self.br.sendTransform(inverted_trans,
                                      orig_rot,
                                      rospy.Time.now(),
                                      "ar_check2",
                                      "web_camera") 

                ## keeps same x,y,z world location, but in a different orientation
                self.br.sendTransform(orig_trans,
                                      inverted_rot,
                                      rospy.Time.now(),
                                      "ar_check3",
                                      "web_camera") 

                ## goes backward in space, with different orientation (same as from ar_check 3)
                self.br.sendTransform(inverted_trans,
                                      inverted_rot,
                                      rospy.Time.now(),
                                      "ar_check4",
                                      "web_camera")              

                
                # Gets the inverse transform just using a lookup...works as expected
                (trans,rot) = self.tf_listener.lookupTransform("ar_marker_1", "web_camera", rospy.Time(0))

                self.br.sendTransform(trans,
                                   rot,
                                   rospy.Time.now(),
                                   "web_camera_original",
                                    "ar_marker_1")

                
                ## Inverse transformation check, works as expected
                matrix_tf = self.tfros.fromTranslationRotation(orig_trans, orig_rot)
                # print "Camera depth to ar marker tf Matrix"
                print matrix_tf

                print "Original trans"
                print orig_trans
                print "Original rotation"
                print orig_rot

                np_inverted = np.linalg.inv(matrix_tf)
                print "Inverted matrix"
                print np_inverted

                inverted_trans = (np_inverted[0,3], np_inverted[1,3], np_inverted[2,3])
                print "Inverted trans:"
                print inverted_trans
                
                inverted_rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, -pose.orientation.w)
                print "inverted rotation:"
                print inverted_rot

                self.br.sendTransform(trans,
                                   rot,
                                   rospy.Time.now(),
                                   "web_camera_inverse_original",
                                    "ar_marker_1")

            


def main():
    rospy.init_node('ar_tf_id_changer', anonymous=True)

    workAround = WorkAround()

    rospy.spin()

if __name__ == '__main__':
    main()