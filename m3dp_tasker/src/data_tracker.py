#!/usr/bin/python
import scipy.io as spio
import os
import rospy
import tf
import time
import sys
import math
import string
import rospkg
import tf2_ros
from copy import deepcopy
from std_msgs.msg import Int32
from m3dp_msgs.srv import String
from m3dp_msgs.msg import TaskTrajectory, TaskPoint, PrintTrajectoryAction, PrintTrajectoryGoal, PrintTrajectoryFeedback, LayerProgress
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Joy
import actionlib
import struct



class DataTracker:
    def __init__(self):        
        self.tracked_poses_pub = rospy.Publisher("data/tracked_poses", PoseStamped, queue_size=10)        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def trans2pose(self,t,name):
        pose=PoseStamped()
        pose.header.frame_id=name
        pose.pose.position.x=t.translation.x
        pose.pose.position.y=t.translation.y
        pose.pose.position.z=t.translation.z
        pose.pose.orientation.x=t.rotation.x
        pose.pose.orientation.y=t.rotation.y
        pose.pose.orientation.z=t.rotation.z
        pose.pose.orientation.w=t.rotation.w
        return pose


    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            try:
                blf = self.tfBuffer.lookup_transform("odom", 'base_link_footprint', rospy.Time())               
                ee = self.tfBuffer.lookup_transform("odom", 'extruder_ee', rospy.Time())
                eegt = self.tfBuffer.lookup_transform("odom", 'as_ee_tracked', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                rospy.logerr_throttle("Data Tracker can't find all frames")
                continue
            
            self.tracked_poses_pub.publish(self.trans2pose(blf,"base_link_footprint"))
            self.tracked_poses_pub.publish(self.trans2pose(ee,"extruder_ee"))
            self.tracked_poses_pub.publish(self.trans2pose(eegt,"as_ee_tracked"))
  
if __name__ == "__main__":
    rospy.init_node('DataTracker')
    pt = DataTracker()
    rospy.spin()
