#!/usr/bin/env python
import h5py
import numpy as np
import rospy
from datetime import datetime
from matplotlib import pyplot as plt
from auxiliar_classes.sensor_class import proximity_sensor
from auxiliar_classes.ross_message import ross_message
from cv_bridge import CvBridge
import cv2
import os.path
import logging

class interface(ross_message):

	def __init__(self):
		rospy.init_node('visual')
		self.name = rospy.get_param('~robot_name')
		self.working_path = rospy.get_param('~data_path')
		self.save_data = bool(int(rospy.get_param('~save_data')))
		self.dataset_size = int(rospy.get_param('~dataset_size'))
		self.debug =  bool(int(rospy.get_param('~debug')))
		self.frames_collected = 0
		self.previous_metadata = None 
		self.offset = 0 #number of pictures already saved
		#Logss
		path = rospy.get_param('~debug_path') + 'interface.log'    
		if self.debug:  
			self.logger = interface.setup_logger('sensor_controller', path,level = logging.DEBUG)
		else:
			self.logger = interface.setup_logger('sensor_controller', path,level = logging.NOTSET)
		self.logger.info('Interface start ' + self.name)
		if(os.path.exists('{}/metadata.csv'.format(self.working_path))):
			self.previous_metadata = np.loadtxt('{}/metadata.csv'.format(self.working_path), delimiter=',')
			self.offset = self.previous_metadata[-1][0]+1


		self.pose_x = []
		self.pose_y = []
		self.init_publisher_subscribers_camera()
		self.init_publisher_subscribers_odometry()
		self.sensor = proximity_sensor()
		self.init_publisher_subscribers_sensors()
		self.rate = rospy.Rate(10)
		
		self.subscriber_names = ["/proximity/left", "/proximity/center_left",
								"/proximity/center", "/proximity/center_right",
								 "/proximity/right"
								]
		self.sensors_names = ['left', 'center_left', 'center', 'center_right',
								'right']
		
		

		self.y = np.empty((self.dataset_size,1))
		self.ang = np.empty((self.dataset_size,1))
		self.id = np.empty((self.dataset_size,1))

		self.metadata = np.empty((self.dataset_size, 3))
		self.sensors_data = np.empty((self.dataset_size, 5))
		self.frame_count = 0
		self.sensors_weight = np.array([-1, -2, 0, 2, 1])


	def update(self):
		sensors = self.sensor
		self.sensors_array = np.array([sensors.proximity_left,
		 sensors.proximity_central_left, sensors.proximity_center,
		 sensors.proximity_central_right, sensors.proximity_right])
		self.proximity_weighted = np.dot(self.sensors_array,
										 self.sensors_weight)
		print("sensors {}".format(self.sensors_array))
		print("y {}".format(self.proximity_weighted))


	def renderInterface(self):
		self.frame_count += 1
		"""
		Render interface
		"""
		fig = plt.figure(1,figsize = (8,6))
		#Set background color
		fig.patch.set_facecolor('#E0E0E0')
		fig.patch.set_alpha(0.7)
		fig.suptitle('Robot ' + self.name, fontsize=20)
		#Velocimeter
		ax1 = plt.subplot(2,2,1)
		speeds = [1,2]
		plt.vlines(0, 0.5, 2.5)
		plt.barh(speeds,[self.velocity.linear.x,self.velocity.angular.z])
		plt.yticks(speeds, ["linear","angular"])
		ax1.title.set_text('Speed ms')

		#Camera
		ax2 = plt.subplot(2,2,2)
		plt.imshow(self.rgb_undist)
		#print('SHAPEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE', self.rgb_undist.shape)
		cmap = plt.cm.jet
		ax2.title.set_text('Camera')

		#Odometry map
		grid = plt.GridSpec(2, 3, wspace=0.4, hspace=0.3)
		plt.subplot(grid[1, :])
		plt.plot(self.pose_x,self.pose_y,'--o')
		plt.plot(self.pose_x[-1],self.pose_y[-1],'rx', label = "last position")
		plt.grid(color='g', linestyle='--', linewidth=0.5)
		plt.xlabel('Odometry', fontsize=14)
		plt.legend()
		#remder 
		fig.canvas.draw()
		plt.show(block=False)

	#end debugging
	def gather_data(self):	
		"""
		Gather data for database
		"""
		is_close = ((np.abs(self.velocity.linear.x) <= 0.01)
					 and 
					 (np.abs(self.velocity.angular.z) >= 0.2))

		mod_condition = self.frame_count % 2 == 0
		frame_condition = self.frame_count % 3 == 1
		frame_condition_y0 = self.frame_count % 5 == 1
		front_colision = self.sensor.front_colision()

		if (  (front_colision 
				and
				self.frames_collected <= self.dataset_size - 1 
				and 
				frame_condition)
			or frame_condition_y0):

			print("Front collision {} --- Image number {}".
				format(front_colision,self.frames_collected+1))
			self.logger.info("Front collision {} --- Image number {}".
				format(front_colision, self.frames_collected+1))
			#saving pictures in a folder
			cv2.imwrite("{}/{}.jpg".format(self.working_path,self.frames_collected+self.offset), self.rgb_undist)
			self.metadata[self.frames_collected][0] = self.frames_collected+self.offset
			self.metadata[self.frames_collected][1] = self.proximity_weighted
			self.metadata[self.frames_collected][2] = self.velocity.angular.z
			self.sensors_data[self.frames_collected] = self.sensors_array

			self.frames_collected += 1

		if self.frames_collected >=self.dataset_size:
			print("******************************************")
			print("{} IMAGEs COMPUTED".format(self.frames_collected))
			print("PROCEDURE COMPLETE")
			print("******************************************")

			if(self.previous_metadata is not None):
				np.savetxt("{}/metadata.csv".format(self.working_path), np.concatenate((self.previous_metadata, self.metadata), axis=0), header='id, y,angular_velocity',delimiter=",")
			else:
				np.savetxt("{}/metadata.csv".format(self.working_path), self.metadata,
					header='id,y,angular_velocity',
					delimiter=",")
				np.savetxt("{}/sensors.csv".format(self.working_path), self.sensors_data,
					header='left,center_left,center,center_right, right', delimiter=",")
			exit()
			

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
			if self.save_data:
				self.update()
				self.gather_data()
			self.rate.sleep()



if __name__ == '__main__':
	
	visual_interface = interface()

	try:
		visual_interface.run()
	except rospy.ROSInterruptException as e:
		pass
