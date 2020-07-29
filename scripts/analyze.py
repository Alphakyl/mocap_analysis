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
        while not rospy.is_shutdown():
            if (self.gate_pose and self.robot_pose and self.leica_transform):
                self.tf_from_mocap = tf.transformations.concatenate_matrices(tf.transformations.inverse_matrix(self.robot_pose),self.sub_robot)
                rospy.loginfo("Robot->Gate:\n%s, %s",\
                tf.transformations.translation_from_matrix(self.tf_from_mocap).__str__(),\
                [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.tf_from_mocap, 'sxyz')].__str__())
                rospy.loginfo("Robot->Gate:\n%s, %s",\
                tf.transformations.translation_from_matrix(self.leica_transform).__str__(),\
                [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.leica_transform, 'sxyz')].__str__())
                



    def sub_gate(self, pose):
        if not self.gate_pose: 
            # Get the translation to the rigid body
            translation_matrix = tf.transformations.translation_matrix([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            # Get the rotation to the rigid body
            rot_matrix = tf.transformations.rotation_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            self.gate_pose = tf.transformations.concatenate_matrices(translation_matrix, rot_matrix)
        
            
        
    def sub_robot(self, pose):
        if not self.robot_pose:
            # Get the translation to the rigid body
            translation_matrix = tf.transformations.translation_matrix([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            # Get the rotation to the rigid body
            rot_matrix = tf.transformations.rotation_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            self.robot_pose = tf.transformations.concatenate_matrices(translation_matrix, rot_matrix)
    
    def sub_leica(self, pose):
        if not self.leica_transform:
            # Get the translation to the rigid body
            translation_matrix = tf.transformations.translation_matrix([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            # Get the rotation to the rigid body
            rot_matrix = tf.transformations.rotation_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            self.robot_pose = tf.transformations.concatenate_matrices(translation_matrix, rot_matrix)
