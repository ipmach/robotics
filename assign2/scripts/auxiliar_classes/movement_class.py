import sys
import json
import numpy as np
import random

class movement():
    """
    Class use in by the controller to decide when a instruction is finish.
    """

    def __init__(self,ini_x,ini_y,ini_angle, end_x,end_y,end_angle):
        self.ini_x = ini_x
        self.ini_y = ini_y
        self.end_x = end_x
        self.end_y = end_y
        ##Angles
        self.tolerance = 0.05
        self.end_angle = end_angle
        self.ini_angle = ini_angle
        ###
        #print('data set ini: x: {0} y {1} angle {2} end: x {3} y {4} angle {5}'.format(self.ini_x,self.ini_y,ini_angle,self.end_x,self.end_y,self.end_angle))

    def correctAngle(self,a):
        """
        Preprocess the angle
        """
        if a <0:
          return abs(a + 6.30)
        return a

    def isover_angles(self,angle,w):
        """
        Check if we got to the desire angle
        """
        if w == 0:
            return True
        angle = self.correctAngle(angle)
        low_range = self.correctAngle(self.end_angle - self.tolerance)
        high_range = self.correctAngle(self.end_angle + self.tolerance) 
        #if self.end_angle == 0:
        #    low_range = 0
        #print('Actual angle: {0}, low range: {1}, high range: {2}'.format(angle,low_range,high_range))
        return True if low_range <= angle <= high_range else False

    def recommendAngular(self):
        """
        Give you the best angular velocity sign
        """
        return 1 if self.correctAngle(self.end_angle) - self.correctAngle(self.ini_angle) > 0 else -1
        

    def isover(self,x,y,angle,w):
        """
        Check if we got to the desire position and angle
        """
        key = [False,False,False]
        if abs(x - self.ini_x) >= self.end_x:
            key[0] = True
        if abs(y - self.ini_y) >= self.end_y:
            key[1] = True
        key[2] = self.isover_angles(angle,w)
        #print(key)
        return all(key)