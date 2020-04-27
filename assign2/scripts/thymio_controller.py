#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion

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
        ###
        print('data set ini: x: {0} y {1} angle {2} end: x {3} y {4} angle {5}'.format(self.ini_x,self.ini_y,ini_angle,self.end_x,self.end_y,self.end_angle))

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
        if self.end_angle == 0:
            low_range = 0
        print('Actual angle: {0}, low range: {1}, high range: {2}'.format(angle,low_range,high_range))
        return True if low_range <= angle <= high_range else False


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
        return all(key)


class ThymioController:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'thymio_controller'  # name of the node
        )

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(
            self.name + '/cmd_vel',  # name of the topic
            Twist,  # message type
            queue_size=10  # queue size
        )

        # create pose subscriber
        self.pose_subscriber = rospy.Subscriber(
            self.name + '/odom',  # name of the topic
            Odometry,  # message type
            self.log_odometry  # function that hanldes incoming messages
        )

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)

    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple."""

        # create a quaternion from the pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        result = (
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result

    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.velocity = data.twist.twist

        printable_pose = self.human_readable_pose2d(self.pose)
        # log robot's pose
        
        rospy.loginfo_throttle(
            period=5,  # log every 10 seconds
            msg=self.name + ' odometry ' +' (%.3f, %.3f, %.3f) ' % printable_pose  # message
        )
        

    def get_control(self,m):
        """
        Controller, assign angular and linear velocity to the robot 
        and decide when the robot need to change instruction
        """
        data = self.human_readable_pose2d(self.pose)
        linear = self.actions[self.actual_action]["linear"]
        angular = self.actions[self.actual_action]["angular"]
        print("Actual instruction: {0}".format(self.actual_action))
        if not m.isover(data[0],data[1],data[2], 1):
            return Twist(
                linear=Vector3(
                    linear,  # moves forward .2 m/s
                    .0,
                    .0,
                ),
                angular=Vector3(
                    .0,
                    .0,
                    angular
                )
            ),False
        else:
            return Twist(
                linear=Vector3(
                    .0,  # moves forward .2 m/s
                    .0,
                    .0,
                ),
                angular=Vector3(
                    .0,
                    .0,
                    .0
                )
            ), True

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
        #Dictionary with the instructions 
        self.actions = {1:{"linear":0.2,"angular":0.4, "x":0 ,"y":0 ,"angle":3},
                   2:{"linear":0.2,"angular": -0.4, "x":0 ,"y":0 ,"angle":0},
                   3:{"linear":0.2,"angular":-0.4, "x":0 ,"y":0 ,"angle":3},
                   4:{"linear":0.2,"angular":0.4, "x":0 ,"y":0 ,"angle":0}}
        #Index of the actual instruction
        self.actual_action = 1

        #Initialize movement
        theta,x,y,_,_ = self.get_theta_coord()
        data = self.human_readable_pose2d(self.pose)
        m = movement(data[0],data[1],data[2],x,y,theta)

        while not rospy.is_shutdown():

            # decide control action
            velocity, isfinish = self.get_control(m)

            # publish velocity message
            self.velocity_publisher.publish(velocity)

            #Change action        
            if isfinish:
                #we check if we need to reset (Make instructions in loop)
                if max(self.actions.keys()) == self.actual_action:
                    self.actual_action = 1
                else:
                    self.actual_action += 1 
                #Initialize again movement
                theta,x,y,_,_ = self.get_theta_coord()
                data = self.human_readable_pose2d(self.pose)
                m = movement(data[0],data[1],data[2],x,y,theta)
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
