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
		self.working_path = rospy.get_param('~data_path')
		#self.model = self.initializeNetwork()
		self.model = tf.keras.models.load_model(
						self.working_path+'/dturn.h5')
		self.init_publisher_subscribers_camera()

		self.velocity_publisher = rospy.Publisher(
			self.name + '/cmd_vel',  # name of the topic
			Twist,  # message type
			queue_size=10  # queue size
		)
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
		frame[0] = self.rgb_undist / 255.0
		
		angular_velocity = self.model.predict(frame) * 10
		print("here is the ang from the model {}".format(angular_velocity))

		return Twist(
			linear=Vector3(
				.12,  # moves forward .2 m/s
				.0,
				.0,
			),
			angular=Vector3(
				.0,
				.0,
				angular_velocity 
			)
		)

	def run(self):
		# i = 0
		while not rospy.is_shutdown():

			#if i % 3 ==0:
				velocity = self.explore()
				self.velocity_publisher.publish(velocity)
				print('ANGULAR VELOCITY PREDICTED {}'.format(velocity.angular.z))
				self.rate.sleep()
			# sleep until next step
		# i += 1

	def stop(self):
		"""Stops the robot."""
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