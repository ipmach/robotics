#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int64
from matplotlib import pyplot as plt


class interface():

	def __init__(self):
		rospy.init_node('visual')
		self.name = rospy.get_param('~robot_name')
		self.pose_x = []
		self.pose_y = []
		self.pose = Pose()
		self.velocity = Twist()
		self.pose_subscriber = rospy.Subscriber(self.name+'/odom', Odometry, self.log_odometry)
		self.flag_subscriber = rospy.Subscriber(self.name+'/flag', Int64, self.log_flag)
		self.flag = 0
		self.rate = rospy.Rate(10)

	def log_odometry(self,data):
		self.pose = data.pose.pose
		self.velocity = data.twist.twist
		
	def log_flag(self,data):
		self.flag = data.data

	def drawRobot(self,x,y):
		"""
		Draw robot
		"""
		plt.plot([x-0.01,x,x+0.01],[y-0.01,y+0.02,y-0.01])

	def renderInterface(self):
		"""
		Render interface
		"""
		fig = plt.figure(1,figsize = (5,5))
		fig.suptitle('Robot ' + self.name, fontsize=20)
		plt.subplot(2,1,1)
		speeds = [1,2]
		plt.vlines(0, 0.5, 2.5)
		plt.barh(speeds,[self.velocity.linear.x,self.velocity.angular.z])
		plt.yticks(speeds, ["linear","angular"])
		plt.xlabel('Speed ms', fontsize=14)
		plt.subplot(2,1,2)
		#self.drawRobot(self.pose_x[-1],self.pose_y[-1])
		plt.plot(self.pose_x,self.pose_y,'-o')
		plt.xlabel('Odometry', fontsize=14)
		fig.canvas.draw()
		plt.show(block=False)

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
			self.rate.sleep()


if __name__ == '__main__':
	
	visual_interface = interface()

	try:
		visual_interface.run()
	except rospy.ROSInterruptException as e:
		pass


