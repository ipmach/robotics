#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range,Image,CameraInfo
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int64
from matplotlib import pyplot as plt
import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
import time


class interface():

	def __init__(self):
		rospy.init_node('visual')
		self.name = rospy.get_param('~robot_name')
		self.pose_x = []
		self.pose_y = []
		self.pose = Pose()
		self.velocity = Twist()
		self.pose_subscriber = rospy.Subscriber(self.name+'/odom', Odometry, self.log_odometry)
		self.flag_subscriber = rospy.Subscriber(self.name+'/flag', Int64, self.log_flag)
		image_sub = message_filters.Subscriber('/thymio10/camera/image_raw', Image)
		info_sub = message_filters.Subscriber('/thymio10/camera/camera_info', CameraInfo)
		ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
		ts.registerCallback(self.callback)
		self.flag = 0
		self.rate = rospy.Rate(10)

	def log_odometry(self,data):
		self.pose = data.pose.pose
		self.velocity = data.twist.twist

	def log_flag(self,data):
		self.flag = data.data

	def callback(self,rgb_msg, camera_info):
	   rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
	   camera_info_K = np.array(camera_info.K).reshape([3, 3])
	   camera_info_D = np.array(camera_info.D)
	   self.rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)
	   #print(np.array(self.rgb_undist).shape)

	def renderInterface(self):
		"""
		Render interface
		"""
		fig = plt.figure(1,figsize = (8,6))
		fig.suptitle('Robot ' + self.name, fontsize=20)
		ax1 = plt.subplot(2,2,1)
		speeds = [1,2]
		plt.vlines(0, 0.5, 2.5)
		plt.barh(speeds,[self.velocity.linear.x,self.velocity.angular.z])
		plt.yticks(speeds, ["linear","angular"])
		ax1.title.set_text('Speed ms')
		#plt.xlabel('Speed ms', fontsize=14)
		ax2 = plt.subplot(2,2,2)
		plt.imshow(self.rgb_undist)
		ax2.title.set_text('Camera')
		grid = plt.GridSpec(2, 3, wspace=0.4, hspace=0.3)
		plt.subplot(grid[1, :])
		plt.plot(self.pose_x,self.pose_y,'--o')
		plt.grid(color='g', linestyle='--', linewidth=0.5)
		plt.xlabel('Odometry', fontsize=14)
		fig.canvas.draw()
		plt.show(block=False)

	def run(self):
		self.pose_x.append(self.pose.position.x)
		self.pose_y.append(self.pose.position.y)
		self.renderInterface()
		while not rospy.is_shutdown():
			if self.flag == 1:
				self.pose_x.append(self.pose.position.x)
				self.pose_y.append(self.pose.position.y)
			plt.clf()
			self.renderInterface()
			self.rate.sleep()
			#time.sleep(1)


if __name__ == '__main__':

	visual_interface = interface()

	try:
		visual_interface.run()
	except rospy.ROSInterruptException as e:
		pass
