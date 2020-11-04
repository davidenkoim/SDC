#! /usr/bin python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import time

class Follower:
    def __init__(self):
        self.x, self.y, self.theta = 0, 0, 0
        self.pub = rospy.Publisher('/leonardo/cmd_vel', Twist, queue_size=1)
        self.msg = Twist()
        rospy.Subscriber('/turtle1/pose', Pose, self.follow)
        rospy.Subscriber('/leonardo/pose', Pose, self.leonardo_pose)
        time.sleep(1)
        
    def leonardo_pose(self, msg):
        self.x, self.y, self.theta = msg.x, msg.y, msg.theta

    def follow(self, msg):
        eps = 1e-1
        x, y, theta = msg.x, msg.y, msg.theta
        
        angle = np.arctan2(self.y - y, self.x - x)
        angle -= self.theta
        w = np.sign(angle)
        if angle > np.pi:
            w = -1
        elif angle < -np.pi:
            w = 1
        self.msg.linear.x = 0.5
        self.msg.angular.z = -w * np.pi / 2.
        if (self.x - x) ** 2 + (self.y - y) ** 2 > eps:
            self.pub.publish(self.msg)
        

rospy.init_node('follower')
f = Follower()
rospy.spin()