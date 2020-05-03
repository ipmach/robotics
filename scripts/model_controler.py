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


class Net(ross_message):

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'thymio_controller'  # name of the node
        )

        #Dictionary with the instructions  and other parameters
        self.name = rospy.get_param('~robot_name')


        # set node update frequency in Hz
        self.rate = rospy.Rate(10)


    def run(self):
        while not rospy.is_shutdown():
            2+2

if __name__ == '__main__':

    n = Net()

    try:
        n.run()
    except rospy.ROSInterruptException as e:
        pass