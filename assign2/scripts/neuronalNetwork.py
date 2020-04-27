#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
import sys
import json
import numpy as np
import random

from movement_class import movement
from sensor_class import proximity_sensor
from std_msgs.msg import Int64

class Net:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'thymio_controller'  # name of the node
        )

        #Dictionary with the instructions  and other parameters
        self.name = rospy.get_param('~robot_name')
        self.use_net = bool(int(rospy.get_param('~use_net')))
        self.train_net =  bool(int(rospy.get_param('~train_net')))

        #Initialize sensors
        self.sensor = proximity_sensor()

        self.flag_publisher = rospy.Publisher(
            self.name + '/flag',  # name of the topic
            Int64,  # message type
            queue_size=10  # queue size
        )

        self.proximity_center_subscriber = rospy.Subscriber(
            self.name + '/proximity/center',  # name of the topic
            Range,  # message type
            self.log_center_range  # function that hanldes incoming messages
        )

        self.proximity_right_subscriber = rospy.Subscriber(
            self.name + '/proximity/right',  # name of the topic
            Range,  # message type
            self.log_right_range  # function that hanldes incoming messages
        )

        self.proximity_left_subscriber = rospy.Subscriber(
            self.name + '/proximity/left',  # name of the topic
            Range,  # message type
            self.log_left_range  # function that hanldes incoming messages
        )

        self.proximity_central_right_subscriber = rospy.Subscriber(
            self.name + '/proximity/center_right',  # name of the topic
            Range,  # message type
            self.log_central_right_range  # function that hanldes incoming messages
        )

        self.proximity_central_left_subscriber = rospy.Subscriber(
            self.name + '/proximity/center_left',  # name of the topic
            Range,  # message type
            self.log_central_left_range  # function that hanldes incoming messages
        )
        self.proximity_rear_right_subscriber = rospy.Subscriber(
            self.name + '/proximity/rear_right',  # name of the topic
            Range,  # message type
            self.log_rear_right_range  # function that hanldes incoming messages
        )

        self.proximity_rear_left_subscriber = rospy.Subscriber(
            self.name + '/proximity/rear_left',  # name of the topic
            Range,  # message type
            self.log_rear_left_range  # function that hanldes incoming messages
        )

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)

    def log_center_range(self,data):
        """
        Update center sensor
        """
        center =  100 * data.range/(data.max_range - data.min_range)
        self.sensor.proximity_center = center

    def log_right_range(self,data):
        """
        Update right sensor
        """
        right =  100 * data.range/(data.max_range - data.min_range)
        self.sensor.proximity_right = right

    def log_left_range(self,data):
        """
        Update left sensor
        """
        left =  100 * data.range/(data.max_range - data.min_range)
        self.sensor.proximity_left = left

    def log_central_right_range(self,data):
        """
        Update central right sensor
        """
        right_central =  100 * data.range/(data.max_range - data.min_range)
        self.sensor.proximity_central_right = right_central

    def log_central_left_range(self,data):
        """
        Update central left sensor
        """
        left_central =  100 * data.range/(data.max_range - data.min_range)
        self.sensor.proximity_central_left = left_central

    def log_rear_right_range(self,data):
        """
        Update rear right sensor
        """
        rear_right =  100 * data.range/(data.max_range - data.min_range)
        self.sensor.proximity_rear_right = rear_right

    def log_rear_left_range(self,data):
        """
        Update rear left sensor
        """
        rear_left =  100 * data.range/(data.max_range - data.min_range)
        self.sensor.proximity_rear_left = rear_left


    def run(self):
        while not rospy.is_shutdown():
            2+2

if __name__ == '__main__':

    n = Net()

    try:
        n.run()
    except rospy.ROSInterruptException as e:
        pass