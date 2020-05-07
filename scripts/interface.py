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
		self.working_path = rospy.get_param('~data_path')
		self.save_data = bool(int(rospy.get_param('~save_data')))
		self.dataset_size = int(rospy.get_param('~dataset_size'))
		
		self.pose_x = []
		self.pose_y = []
		self.init_publisher_subscribers_camera()
		self.init_publisher_subscribers_odometry()
		self.sensor = proximity_sensor()
		self.init_publisher_subscribers_sensors()
		self.rate = rospy.Rate(10)
		
		self.subscriber_names = ["/proximity/left", "/proximity/center_left",
								"/proximity/center", "/proximity/center_right", "/proximity/right"
								]
		self.sensors_names = ['left', 'center_left', 'center', 'center_right',
								'right']
		
		self.frames_collected = 0
		self.chunck_number = 0
		self.chunck_size = 1000
		#just for debugg
		self.lastprint = 0
		#end debug

		if self.save_data:
			self.initializeDataset()

	   #print(np.array(self.rgb_undist).shape)

	def initializeDataset(self):
		remaining = self.dataset_size-self.frames_collected
		size = self.chunck_size
		if(remaining > size):
			size = remaining

		today_str = datetime.today().strftime("%Y-%m-%d-%H-%M-%S")
		self.data_file = h5py.File("{}/rgb_96x128_training_data_{}.h5".format(self.working_path,
																  today_str))
		#self.dataset = self.data_file.create_dataset('train',
				#				(self.dataset_size, 480, 640, 3), 'f')
		self.dataset = self.data_file.create_dataset('train',
								(size,) + self.rgb_undist.shape, 'f', compression="gzip")
		#l[0] + (l[1],)
		self.dataset_y = self.data_file.create_dataset('train_y',
								(size, 1), 'f', compression="gzip")
		self.dataset_ang = self.data_file.create_dataset('train_ang',
								(size, 1), 'f', compression="gzip")

		self.data_counter = 0
		self.frame_count = 0
		self.raw_data = []
		self.sensors_array = np.zeros((5,))
		self.sensors_weight = np.array([-1, -2, 0, 2, 1])
		self.proximity_weighted = 0

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

	#just for debugging
	def printStatus(self,front_colision):
		print("--------------------------------------------------")
		print("Front colision {}".format(front_colision))
		print("Chunck number {} filled with {} ".format(self.chunck_number+1, self.data_counter+1))
		print("Total number of images computed {}".format(self.frames_collected+1))

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
		front_colision = self.sensor.front_colision()
		#print("Front colision {}".format(front_colision))
		if (front_colision and self.data_counter <= self.dataset_size - 1 and frame_condition):
			self.printStatus(front_colision)
			#print("Too close! should capture")
			self.raw_data.append(self.rgb_undist)
			self.dataset[self.data_counter] = self.rgb_undist
			self.dataset_y[self.data_counter] = self.proximity_weighted
			self.dataset_ang[self.data_counter] = self.velocity.angular.z
			self.data_counter += 1
			self.frames_collected += 1
		elif frame_condition:
			self.printStatus(front_colision)
			self.dataset[self.data_counter] = self.rgb_undist
			self.dataset_y[self.data_counter] = self.proximity_weighted
			self.dataset_ang[self.data_counter] = self.velocity.angular.z
			self.raw_data.append(self.rgb_undist)
			self.data_counter += 1
			self.frames_collected += 1

		#print("Chunck number {} filled with {} ".format(self.chunck_number+1, self.data_counter))
		#print("Total number of images computed {}".format(self.frames_collected))


		if self.frames_collected >=self.dataset_size:
			print("******************************************")
			print("******************************************")
			print("")
			print("CHUNCK NUMBER {} COMPLETED".format(self.chunck_number+1))
			print("PROCEDURE ACCOMPLISHED")
			print("")
			print("******************************************")
			print("******************************************")

			#print(type(self.raw_data))
			self.data_file.close()
			exit()
		if self.data_counter >=self.chunck_size:
			print("******************************************")
			print("")
			print("CHUNCK NUMBER {} COMPLETED".format(self.chunck_number+1))
			print("")
			print("******************************************")

			#print(type(self.raw_data))
			self.data_file.close()
			self.initializeDataset()
			self.chunck_number += 1

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


