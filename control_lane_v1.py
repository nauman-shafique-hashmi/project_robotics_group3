#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
class TurtleBot3Aruco:
    def __init__(self):
        rospy.init_node('turtlebot3_aruco', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)
        #self.twist_pub = rospy.Publisher('/control/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()
        self.aruco_detected = False
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)#######################modify to real topic
        self.sub_lane = rospy.Subscriber('/control/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.lastError = 0
        self.MAX_VEL = 0.1
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 使用ArUco检测
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None and len(ids) > 0:
                self.aruco_detected = True
                rospy.loginfo("ArUco detected!")
            else:
                self.aruco_detected = False

        except Exception as e:
            rospy.logerr(e)

    def stop_turtlebot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.pub_cmd_vel.publish(twist_msg)

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        
        twist = Twist()
        # twist.linear.x = 0.05        
        twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.aruco_detected:
                rospy.loginfo("Stopping for 3 seconds...")
                self.stop_turtlebot()
                rospy.sleep(3)
                rospy.loginfo("Resuming following the line...")
                self.aruco_detected = False
                rospy.on_shutdown(self.fnShutDown)           

            rate.sleep()

if __name__ == '__main__':
    try:
        turtlebot3_aruco = TurtleBot3Aruco()
        turtlebot3_aruco.run()
    except rospy.ROSInterruptException:
        pass
