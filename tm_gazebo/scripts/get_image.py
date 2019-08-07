#!/usr/bin/env python
from __future__ import print_function

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from argparse import ArgumentParser
import numpy as np

parser = ArgumentParser()
parser.add_argument("--type", "-t", help="Specify the image type.", default="color")

class ImageReader:

	def __init__(self, image_type):
		
		if image_type == "depth":
			image_topic = '/r200/camera/depth/image_raw'
			self.save_dir = '/tmp/realsense_images/depth/'
			self.encoding = '32FC1'
		else:
			image_topic = '/r200/camera/color/image_raw'
			self.save_dir = '/tmp/realsense_images/rgb/'
			self.encoding = 'bgr8'
			
		self.bridge = CvBridge()
		self.image_subscriber = rospy.Subscriber(image_topic, Image, self.callback)
		self.image_type = image_type

	def callback(self, msg):
		
		print ("Received an image!")
		try:
			
			cv2_img = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
		except CvBridgeError, e:
			
			print (e)
		else:
			
			time_stamp = time.strftime("%Y%b%d_%H_%M_%S", time.localtime())
				
			if not os.path.exists(self.save_dir):
				
				os.makedirs(self.save_dir)
				
				print ("Directory ", self.save_dir, " Created!")
			else:
				print ("Directory ", self.save_dir, " already exists!")
				
			if self.image_type == 'depth':
				final_img = cv2_img
				cv2.imwrite(self.save_dir + 'camera_image_' + time_stamp + '.png', final_img)
			else:
				final_img = cv2_img
				cv2.imwrite(self.save_dir + 'camera_image_' + time_stamp + '.jpeg', final_img)

def main(args):
	
	rospy.init_node('image_listener')
	

	IR = ImageReader(args.type)
	
	rospy.spin()

if __name__ == '__main__':
	args = parser.parse_args()
	main(args)
