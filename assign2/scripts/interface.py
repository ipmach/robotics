#!/usr/bin/env python
import rospy
from matplotlib import pyplot as plt
from auxiliar_classes.ross_message import ross_message

class interface(ross_message):

	def __init__(self):
		rospy.init_node('visual')
		self.name = rospy.get_param('~robot_name')
		self.pose_x = []
		self.pose_y = []
		self.init_publisher_subscribers_camera()
		self.init_publisher_subscribers_odometry()
		self.rate = rospy.Rate(10)

	   #print(np.array(self.rgb_undist).shape)

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
			#time.sleep(1)


if __name__ == '__main__':
	
	visual_interface = interface()

	try:
		visual_interface.run()
	except rospy.ROSInterruptException as e:
		pass


