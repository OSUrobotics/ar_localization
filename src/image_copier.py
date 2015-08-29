#!/usr/bin/env python
'''
Script to get webcam image and image_info topics renamed with a new tf id reference in the header.

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

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import numpy as np
from numpy.linalg import inv
import copy

class ImageCopy():
	def __init__(self):
		self.camera_image_sub = rospy.Subscriber('/webcam/image_raw', Image, self.image_raw_cb)
		self.new_camera_image_pub = rospy.Publisher('/copied_webcam/image_raw', Image, queue_size=10)

		self.camera_info_sub = rospy.Subscriber('/webcam/camera_info', CameraInfo, self.camera_info_cb)
		self.new_camera_info_pub = rospy.Publisher('/copied_webcam/camera_info', CameraInfo, queue_size=10)

	def image_raw_cb(self, image_raw_data):
		new_image_raw_data = copy.copy(image_raw_data)
		new_image_raw_data.header.frame_id = "copied_web_camera" 

		self.new_camera_image_pub.publish(new_image_raw_data)

	def camera_info_cb(self, camera_info_data):
		new_camera_info_data = copy.copy(camera_info_data)
		new_camera_info_data.header.frame_id = "copied_web_camera"

		self.new_camera_info_pub.publish(new_camera_info_data)



def main():
    rospy.init_node('image_copier', anonymous=True)

    image_copy = ImageCopy()

    rospy.spin()

if __name__ == '__main__':
    main()