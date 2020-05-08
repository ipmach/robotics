import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int64
from sensor_msgs.msg import Range,Image,CameraInfo
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
from auxiliar_classes.sensor_class import proximity_sensor


class ross_message:

    def init_publisher_subscribers_sensors(self):
        # create pose subscriber
        self.sensor = proximity_sensor()
        self.pose_subscriber = rospy.Subscriber(
            self.name + '/odom',  # name of the topic
            Odometry,  # message type
            self.log_odometry  # function that hanldes incoming messages
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
    def init_publisher_subscribers_odometry(self, publisher = True, subscriber = True):
        # create velocity publisher
        if publisher:
            self.velocity_publisher = rospy.Publisher(
                self.name + '/cmd_vel',  # name of the topic
                Twist,  # message type
                queue_size=10  # queue size
            )

            self.flag_publisher = rospy.Publisher(
                self.name + '/flag',  # name of the topic
                Int64,  # message type
                queue_size=10  # queue size
            )
        self.flag = 0
        self.pose = Pose()
        self.velocity = Twist()
        if subscriber:
            self.pose_subscriber = rospy.Subscriber(self.name+'/odom', Odometry, self.log_odometry)
            self.flag_subscriber = rospy.Subscriber(self.name+'/flag', Int64, self.log_flag)

    def init_publisher_subscribers_camera(self):
        image_sub = message_filters.Subscriber(self.name+'/camera/image_raw', Image)
        info_sub = message_filters.Subscriber(self.name+'/camera/camera_info', CameraInfo)
        self.rgb_undist = np.zeros((96,128,3)) #IT WAS (600,600)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
        ts.registerCallback(self.log_camera)

    def log_camera(self,rgb_msg, camera_info):
       """
        Updates the image from robot camera
       """
       rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8") #shape 480*640
       camera_info_K = np.array(camera_info.K).reshape([3, 3])
       camera_info_D = np.array(camera_info.D)
       #frame = cv2.cvtColor(cv2.undistort(rgb_image, camera_info_K, camera_info_D), cv2.COLOR_RGB2GRAY)
       frame = cv2.undistort(rgb_image, camera_info_K, camera_info_D)
       self.rgb_undist = cv2.resize(frame, dsize=(128, 96), interpolation=cv2.INTER_CUBIC)

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
        
    def log_flag(self,data):
        self.flag = data.data

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
        
    def log_center_range(self,data):
        """
        Update center sensor
        """
        center =  100 * data.range/(data.max_range - data.min_range)
        self.sensor.proximity_center = center
        rospy.loginfo_throttle(
            period=5,  # log every 10 seconds
            msg=self.name + ' sensors range ' + self.sensor.toString()   # message
        )

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