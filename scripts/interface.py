#!/usr/bin/env python
import h5py
import numpy as np
import rospy
from datetime import datetime
from matplotlib import pyplot as plt
from auxiliar_classes.sensor_class import proximity_sensor
from auxiliar_classes.ross_message import ross_message

class interface(ross_message):

	def __init__(self):
		rospy.init_node('visual')
		self.name = rospy.get_param('~robot_name')
		working_path = rospy.get_param('~data_path')
		self.save_data = bool(int(rospy.get_param('~save_data')))
		self.dataset_size = int(rospy.get_param('~dataset_size'))
		self.pose_x = []
		self.pose_y = []
		self.init_publisher_subscribers_camera()
		self.init_publisher_subscribers_odometry()
		self.sensor = proximity_sensor()
		self.init_publisher_subscribers_sensors()
		self.rate = rospy.Rate(10)
		if self.save_data:
			today_str = datetime.today().strftime("%Y-%m-%d-%H-%M-%S")
			self.data_file = h5py.File("{}/training_data_{}.h5".format(working_path,
																	  today_str))
			self.dataset = self.data_file.create_dataset('train',
									(self.dataset_size, 480, 640, 3), 'f')
			self.dataset_y = self.data_file.create_dataset('train_y',
									(self.dataset_size, 1), 'f')
			self.dataset_ang = self.data_file.create_dataset('train_ang',
									(self.dataset_size, 1), 'f')
			
			self.subscriber_names = ["/proximity/left", "/proximity/center_left",
									"/proximity/center", "/proximity/center_right", "/proximity/right"
									]
			self.sensors_names = ['left', 'center_left', 'center', 'center_right',
									'right']
			self.data_counter = 0
			self.frame_count = 0
			self.raw_data = []
			self.sensors_array = np.zeros((5,))
			self.sensors_weight = np.array([-1, -2, 0, 2, 1])
			self.proximity_weighted = 0

	   #print(np.array(self.rgb_undist).shape)

	def update(self):
		self.frame_count += 1
		sensors = self.sensor
		self.sensors_array = np.array([sensors.proximity_left, sensors.proximity_central_left,
									   sensors.proximity_center, 
									   sensors.proximity_central_right, sensors.proximity_right])
		self.proximity_weighted = np.dot(self.sensors_array, self.sensors_weight)


	def renderInterface(self):
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

	def gather_data(self):	
		"""
		Gather data for database
		"""
		is_close = ((np.abs(self.velocity.linear.x) <= 0.01)
					 and 
					 (np.abs(self.velocity.angular.z) >= 0.2))

		mod_condition = self.frame_count % 2 == 0
		frame_condition = self.frame_count % 3 == 1
		front_colision = self.sensor.front_colision()
		print("front colision {}".format(front_colision))
		if (front_colision and 
			self.data_counter <= self.dataset_size - 1 and
			frame_condition):
			print("too close! should capture")
			self.raw_data.append(self.rgb_undist)
			self.dataset[self.data_counter] = self.rgb_undist
			self.dataset_y[self.data_counter] = self.proximity_weighted
			self.dataset_ang[self.data_counter] = self.velocity.angular.z
			self.data_counter += 1
		elif frame_condition:
			self.dataset[self.data_counter] = self.rgb_undist
			self.dataset_y[self.data_counter] = self.proximity_weighted
			self.dataset_ang[self.data_counter] = self.velocity.angular.z
			self.raw_data.append(self.rgb_undist)
			self.data_counter += 1

		print("images so far {}".format(self.data_counter))
		if self.data_counter >=self.dataset_size:
			print("full of  images!!!!")
			print(type(self.raw_data))
			self.data_file.close()
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
			#time.sleep(1)


if __name__ == '__main__':
	
	visual_interface = interface()

	try:
		visual_interface.run()
	except rospy.ROSInterruptException as e:
		pass
