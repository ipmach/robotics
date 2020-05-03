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

class ThymioController(ross_message):

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'thymio_controller'  # name of the node
        )

        #Dictionary with the instructions  and other parameters
        with open(rospy.get_param('~instruction_path')) as f: #Open recorded data
            self.actions = json.load(f)
        self.random_noise = bool(int(rospy.get_param('~random_noise')))
        self.use_sensors =  bool(int(rospy.get_param('~use_sensors')))
        self.name = rospy.get_param('~robot_name')

        #Initialize publisher and subscribers
        self.init_publisher_subscribers_sensors()
        self.init_publisher_subscribers_odometry()

        #Initialize sensors
        self.sensor = proximity_sensor()

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        self.states = ["Following instruction", "Colision detected","Turning away","Avoiding colision"]

        self.actual_state = self.states[0]

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)

    def get_control(self,m):
        """
        Controller, assign angular and linear velocity to the robot 
        and decide when the robot need to change instruction
        """
        data = self.human_readable_pose2d(self.pose)
        linear = self.actions[self.actual_action]["linear"]
        angular = self.actions[self.actual_action]["angular"]
        if self.aletory:
            linear = 0
            angular =  self.random_angular 

        #Detect an inminent colision States: "Following instruction"  -> "Colision detected"
        if  self.use_sensors and self.actual_state == self.states[0] and self.sensor.front_colision():
            self.actual_state = self.states[1] 
            return Twist(linear=Vector3(.0,.0,.0,),angular=Vector3(.0,.0,.0)),False 

        #Pointing to colision States: "Colision detected" -> "Colision detected" || "Turning away"
        if self.actual_state == self.states[1]:
            if self.sensor.fron_minimun_sensor() < 0:
                return Twist(linear=Vector3(.0,.0,.0,),angular=Vector3(.0,.0,-.25)),False
            if self.sensor.fron_minimun_sensor() > 0:
                return Twist(linear=Vector3(.0,.0,.0,),angular=Vector3(.0,.0,.25)),False
            if self.sensor.fron_minimun_sensor() == 0:
                self.actual_state = self.states[2]
                theta2 = movement.difference_angle(data[2],3.2)
                self.m2 = movement(data[0],data[1],data[2],0,0,theta2)
                #To get a little closer to the wal so we can use better the back sensors
                return Twist(linear=Vector3(.3,.0,.0,),angular=Vector3(.0,.0,.0)), True
          
        #Turning away orthogonal to colision States: "Turning away" -> "Turning away" || "Avoiding colision"       
        if self.actual_state == self.states[2]: 
            if not self.sensor.back_equal_sensor():
            #print(self.m2.isover(data[0],data[1],data[2], angular)) #Maybe in the future
            #if not self.m2.isover(data[0],data[1],data[2], angular):
            #   print(data[2],self.m2.end_angle)
                return Twist(linear=Vector3(.0,.0,.0,),angular=Vector3(.0,.0,.25)),False   
            self.actual_state = self.states[3]

        #Add instruction to run away from colision States: "Avoiding colision" -> "Following instruction"
        if self.actual_state == self.states[3]:
            data = self.human_readable_pose2d(self.pose)
            m = movement(data[0],data[1],data[2],2,0,0)
            if self.actual_action == "1":
                self.actual_action = str(max(np.array(self.actions.keys()).astype(np.int)))
            else:
                self.actual_action = str(int(self.actual_action) -1)
            self.actual_state = self.states[0]
            self.flag_on = True
            print("FLAAAG ACTIVATE ", self.flag_on)

        #Following and changing instructions States: "Following instruction"  -> "Following instruction"
        if not m.isover(data[0],data[1],data[2], angular):
            return Twist(linear=Vector3(linear,.0,.0,),angular=Vector3(.0,.0,angular)),False
        else:
            return Twist(linear=Vector3(.0,.0,.0,),angular=Vector3(.0,.0,.0)), True

    def aleatory_controller(self,m):
        """
        Controller use in the bonus part to create noise
        """
        dec = np.random.choice([1,0],p=[0.0025,0.9975]) #decide if we make noise or not
        #To make noise two conditions need to happen:
        #   First we are not already following a noise instruction
        #   Second actual state is following instructions, we dont create noise in collisions 
        if dec and self.aletory == False and self.actual_state == self.states[0]:
            data = self.human_readable_pose2d(self.pose)
            theta = np.random.choice([1,-1],p=[0.5,0.5]) * random.randint(0, 60) *0.1
            m = movement(data[0],data[1],data[2],0,0,theta)
            self.random_angular = 0.3 * m.recommendAngular()
            if self.actual_action == "1":
                self.actual_action = str(max(np.array(self.actions.keys()).astype(np.int)))
            else:
                self.actual_action = str(int(self.actual_action) -1)
            self.aletory = True
        return m


    def get_theta_coord(self):
         """
         Obtain the data from the self.actions dictionary
         """
         theta = self.actions[self.actual_action]["angle"]
         x = self.actions[self.actual_action]["x"]
         y = self.actions[self.actual_action]["y"]
         linear = self.actions[self.actual_action]["linear"]
         angular = self.actions[self.actual_action]["angular"]
         return theta, x,y,linear,angular

    def run(self):
        """Controls the Thymio."""
        #Index of the actual instruction
        self.actual_action = str(1)
        self.aletory = False
        #Initialize movement
        theta,x,y,_,_ = self.get_theta_coord()
        data = self.human_readable_pose2d(self.pose)
        m = movement(data[0],data[1],data[2],x,y,theta)

        while not rospy.is_shutdown():
            self.flag_on = False
            # decide control action
            velocity, isfinish = self.get_control(m)
            
            if self.random_noise:
                m = self.aleatory_controller(m)

            self.velocity_publisher.publish(velocity)

            if self.flag_on:
                print("FLAAAG ACTIVATE CHECK ", self.flag_on)
                self.flag_publisher.publish(1)

            # publish velocity message
            if self.actual_state != self.states[0] or self.flag_on:
                self.flag_publisher.publish(1)
                self.flag_on = True
            #Change action        
            if isfinish:
                self.aletory = False
                self.flag_on = True
                self.flag_publisher.publish(1)
                #we check if we need to reset (Make instructions in loop)
                if str(max(np.array(self.actions.keys()).astype(np.int))) == self.actual_action:
                    self.actual_action = str(1)
                else:
                    self.actual_action = str(int(self.actual_action) +  1 )
                #Initialize again movement
                theta,x,y,_,_ = self.get_theta_coord()
                data = self.human_readable_pose2d(self.pose)
                m = movement(data[0],data[1],data[2],x,y,theta)

            if not self.flag_on : #Return 0 if the flag_on was not use
                self.flag_publisher.publish(0)
            # sleep until next step
            self.rate.sleep()

    def stop(self):
        """Stops the robot."""

        self.velocity_publisher.publish(
            Twist()  # set velocities to 0
        )

        self.rate.sleep()


if __name__ == '__main__':

    controller = ThymioController()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass