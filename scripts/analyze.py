#!/usr/bin/python2
import rospy 
import tf
import numpy as np 
from geometry_msgs.msg import *

class Analysis:
    def __init__(self,robot_ns):
        self.sub_gate = rospy.Subscriber('/vrpn_client_node/small_gate/pose', PoseStamped, self.gate_callback)
        self.sub_robot = rospy.Subscriber('/vrpn_client_node/small_ugv/pose', PoseStamped, self.robot_callback)
        self.sub_leica = rospy.Subscriber('/leica/robot_to_origin_transform', TransformStamped, self.leica_callback)
        
        self.gate_pose = None
        self.robot_pose = None
        self.leica_transform = None
        if (self.gate_pose and self.robot_pose and robot_pose):
            pose_diff = self.pose_diff()



    def sub_gate(self, pose):
        if not self.gate_pose: 
            # Get the translation to the rigid body
            translation_matrix = tf.transformations.translation_matrix([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            # Get the rotation to the rigid body
            rot_matrix = tf.transformations.rotation_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            # Deal with the offset to the center of the rigid body


        else:
            # moving average stuff


    def sub_robot(self, pose):
        # Get the translation to the rigid body
        translation_matrix = tf.transformations.translation_matrix([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        # Get the rotation to the rigid body
        rot_matrix = tf.transformations.rotation_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        # Deal with the offset to the center of the rigid body

        
    def pose_diff(self, pose1, pose2):
