#!/usr/bin/python2
import rospy 
import tf
import numpy as np 
from geometry_msgs.msg import *
import math
import sys

class Analysis:
    def __init__(self):
        self.sub_gate = rospy.Subscriber('/vrpn_client_node/plate_1/pose', PoseStamped, self.gate_callback)
        self.sub_robot = rospy.Subscriber('/vrpn_client_node/plate_2/pose', PoseStamped, self.robot_callback)
        self.sub_leica = rospy.Subscriber('/leica/robot_to_origin_transform', TransformStamped, self.leica_callback)
        
        self.gate_pose = None
        self.robot_pose = None
        self.leica_transform = None
        while not rospy.is_shutdown():
            if ((self.gate_pose is not None) and (self.robot_pose is not None) and (self.leica_transform is not None)):
                self.tf_from_mocap = tf.transformations.concatenate_matrices(tf.transformations.inverse_matrix(self.robot_pose),self.gate_pose)
                rospy.loginfo("Robot->Gate (Mocap):\n%s, %s",\
                tf.transformations.translation_from_matrix(self.tf_from_mocap).__str__(),\
                [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.tf_from_mocap, 'sxyz')].__str__())
                rospy.loginfo("Robot->Gate (Leica):\n%s, %s",\
                tf.transformations.translation_from_matrix(self.leica_transform).__str__(),\
                [elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.leica_transform, 'sxyz')].__str__())
                translation_error = tf.transformations.translation_from_matrix(self.tf_from_mocap)-tf.transformations.translation_from_matrix(self.leica_transform)
                rospy.loginfo("Translation Error:\n%s", translation_error.__str__())
                rotation_error = np.array([elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.tf_from_mocap, 'sxyz')])-np.array([elem*180/3.14 for elem in tf.transformations.euler_from_matrix(self.leica_transform, 'sxyz')])
                rospy.loginfo("Rotation Error:\n%s", rotation_error.__str__())
                sys.exit("TF Found")



    def gate_callback(self, pose):
        if self.gate_pose is None: 
            # Get the translation to the rigid body
            translation_matrix = tf.transformations.translation_matrix([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            # Get the rotation to the rigid body
            rot_matrix = tf.transformations.quaternion_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            self.gate_pose = tf.transformations.concatenate_matrices(translation_matrix, rot_matrix)
            # # Apply rotations to get frame in correct orientation
            # xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
            # Rx = tf.transformations.rotation_matrix(-math.pi/2.0,xaxis)
            # Ry = tf.transformations.rotation_matrix(-math.pi/2.0,yaxis)
            # Rz = tf.transformations.rotation_matrix(0.0, zaxis)
            # applied_rot_matrix = tf.transformations.concatenate_matrices(Ry,Rx,Rz)
            # self.gate_pose = tf.transformations.concatenate_matrices(self.gate_pose,applied_rot_matrix)
        
            
        
    def robot_callback(self, pose):
        if self.robot_pose is None:
            # Get the translation to the rigid body
            translation_matrix = tf.transformations.translation_matrix([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            # Get the rotation to the rigid body
            rot_matrix = tf.transformations.quaternion_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            self.robot_pose = tf.transformations.concatenate_matrices(translation_matrix,rot_matrix)
            # # Apply rotations to get frame in correct orientation
            # xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
            # Rx = tf.transformations.rotation_matrix(-math.pi/2.0,xaxis)
            # Ry = tf.transformations.rotation_matrix(-math.pi/2.0,yaxis)
            # Rz = tf.transformations.rotation_matrix(0.0, zaxis)
            # applied_rot_matrix = tf.transformations.concatenate_matrices(Ry,Rx,Rz)
            # self.robot_pose = tf.transformations.concatenate_matrices(self.robot_pose,applied_rot_matrix)
    
    def leica_callback(self, pose):
        if self.leica_transform is None:
            # Get the translation to the rigid body
            translation_matrix = tf.transformations.translation_matrix([pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z])
            # Get the rotation to the rigid body
            rot_matrix = tf.transformations.quaternion_matrix([pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w])
            self.leica_transform = tf.transformations.concatenate_matrices(translation_matrix, rot_matrix)

def main():
    rospy.init_node('analyze')
    anal = Analysis()
    rospy.spin()

if __name__ == '__main__':
#    main()
    try:
    	main()
    except rospy.ROSInterruptException:
	pass
