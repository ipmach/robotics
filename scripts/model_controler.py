#!/usr/bin/env python
import rospy
import sys
import json
import numpy as np
import random
from geometry_msgs.msg import Pose, Twist, Vector3
from auxiliar_classes.movement_class import movement
from auxiliar_classes.sensor_class import proximity_sensor
from auxiliar_classes.ross_message import ross_message
import logging

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Activation, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
#from keras.models import load_model

class CNNController(ross_message):

	def __init__(self):
		"""Initialization."""

		# initialize the node
		rospy.init_node(
		    'thymio_controller'  # name of the node
		)

		#Dictionary with the instructions  and other parameters
		self.name = rospy.get_param('~robot_name')
		self.working_path = rospy.get_param('~model_path')
		self.debug =  bool(int(rospy.get_param('~debug')))
		path = rospy.get_param('~debug_path') + 'cnn_control_sensor.log' 
		if self.debug:  
			self.logger = CNNController.setup_logger('sensor_controller', path,level = logging.DEBUG)
		else:
			self.logger = CNNController.setup_logger('sensor_controller', path,level = logging.NOTSET)
		self.logger.info('Robot start moving ' + self.name)
		#self.model = self.initializeNetwork()
		self.model = tf.keras.models.load_model(self.working_path+'/model.h5')
		self.init_publisher_subscribers_camera()
		self.init_publisher_subscribers_odometry()
		self.init_publisher_subscribers_sensors()
		self.sensor = proximity_sensor()

		rospy.on_shutdown(self.stop)
		# set node update frequency in Hz
		self.rate = rospy.Rate(10)



	def initializeNetwork(self):
		# bryan model

		keras.backend.clear_session()
		#Creation of keras sequence CNN
		model = Sequential()

		#Layer 1 and 2 filtering 3x3x32 + maxpooling 3x3 + dropout
		model.add(Conv2D(8, kernel_size=5, activation='relu', padding='same', input_shape=(96, 128, 3)))
		model.add(MaxPooling2D(pool_size=(2, 2)))

		#Layer 3 and 4 filtering 3x3x64 + maxpooling 3x3 + dropout
		model.add(Conv2D(16, kernel_size=16, padding='same', activation='relu'))
		model.add(MaxPooling2D(pool_size=(2, 2)))

		#flatten and fully connected layer
		model.add(Flatten())
		model.add(Dense(16, activation='tanh'))
		model.add(Dropout(rate=0.3))

		model.add(Dense(1, activation='tanh'))

		# opt = keras.optimizers.RMSprop(learning_rate = learn_rate, decay=1e-6)
		# opt = keras.optimizers.Adam(learning_rate=learn_rate)
		opt = keras.optimizers.SGD( learning_rate=0.001, momentum=0.1, nesterov=False, name="SGD" )
		model.compile(loss='mean_absolute_error', optimizer=opt, metrics=['mean_squared_error'])

		return model


	def explore(self):

		frame = np.empty((1,96,128,3))
		frame[0] = self.rgb_undist
		#print(np.array(frame).shape)
		a,b = self.sensor.blind_spot_colision()		#Blind spots
		if a: #Blind spot found 
			self.logger.info('Blind spot found, angular correction {}'.format(b))
			return Twist(linear=Vector3(.1,.0,.0,),angular=Vector3(.0,.0,b )) 

		angular_velocity = self.model.predict(frame/255.)  * -10
		self.logger.debug('angular velocity: {}'.format(angular_velocity) )
		return Twist(linear=Vector3(.1,.0,.0,),angular=Vector3(.0,.0,angular_velocity))

	def run(self):
		flag_iter = 10
		i = 0
		while not rospy.is_shutdown():
			velocity = self.explore()
			self.velocity_publisher.publish(velocity)
			i +=1
			if i== flag_iter:
				i = 0
				self.flag_publisher.publish(1)
			else:
				self.flag_publisher.publish(0)
			# sleep until next step
			self.rate.sleep()

	def stop(self):
		"""Stops the robot."""
		self.logger.info("Robot stop")
		self.velocity_publisher.publish(
			Twist()  # set velocities to 0
		)
		self.rate.sleep()

if __name__ == '__main__':

	controller = CNNController()

	try:
		controller.run()
	except rospy.ROSInterruptException as e:
		pass