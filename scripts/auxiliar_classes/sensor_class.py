import sys
import json
import numpy as np
import random

class proximity_sensor():
    """
    Save the data from the sensors
    """
    #Sensors data
    proximity_center = 100 
    proximity_right = 100
    proximity_left = 100
    proximity_rear_right = 100 
    proximity_rear_left = 100
    proximity_central_right = 100 
    proximity_central_left = 100
    threshold = 95 #Distance tolerance to hit colision
    threshold_min = 45 #Distance tolerance to hit colision
    max_unit = 108 #Number that we ignore reading


    def blind_spot_colision(self):
        """
        Detect blind spot colisions for the camera 
        """
        if self.proximity_left <= self.threshold_min:
            return True, -1
        if self.proximity_right <= self.threshold_min:
            return True, 1
        return False,0

    def front_colision(self):
        """
        Detect a front colision
        """
        return True if self.proximity_center <= self.threshold or \
                       self.proximity_left <= self.threshold or self.proximity_central_left <= self.threshold or \
                       self.proximity_right <= self.threshold or self.proximity_central_right <= self.threshold  else False

    def fron_minimun_sensor(self):
        """
        Return the sensor with minimun data from the front
        """
        aux = np.argmin([self.proximity_right,self.proximity_central_right,self.proximity_center,self.proximity_left,self.proximity_central_left])
        return [-1,-2,0,2,1][aux]

    def back_equal_sensor(self):
        """
        Return True if back sensors have similar data and small than threshold
        """
        if self.proximity_rear_right > self.threshold and self.proximity_rear_left > self.threshold:
            return False
        return abs(self.proximity_rear_right - self.proximity_rear_left) < 2

    def toString(self):
        """
        Return a string of the data
        """
        return "center: {0}, right: {1}, left: {2}, central right {5}, central left {6} ,rear right {3}, rear left {4}".format(self.proximity_center,self.proximity_right,
                self.proximity_left, self.proximity_rear_right,self.proximity_rear_left,self.proximity_central_right,self.proximity_central_left) 
