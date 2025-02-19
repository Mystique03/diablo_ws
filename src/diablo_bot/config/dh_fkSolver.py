#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_matrix
import numpy as np
from math import cos, sin, pi

class FKPublisher(Node):
    def __init__(self):
        super().__init__('fk_publisher')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/end_effector_pose',
            10)
        
        # DH parameters (in cm)
        self.a1 = 5.16
        self.a2 = 16.31
        self.a3 = 19.8
        
        # Initialize joint angles
        self.joint_angles = {
            'r_sho_pitch': 0.0,
            'r_sho_roll': 0.0,
            'r_el': 0.0,
            'r_wrist': 0.0
        }

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_angles:
                self.joint_angles[name] = position
                
        # Calculate forward kinematics
        pose = self.calculate_fk()
        self.publish_pose(pose)

    def calculate_fk(self):
        t1 = self.joint_angles['r_sho_pitch']
        t2 = self.joint_angles['r_sho_roll']
        t3 = self.joint_angles['r_el']

        DH = [
            [t1 - pi/2, pi/2, 0, self.a1],
            [t2, -pi/2, self.a2, 0],
            [t3, 0, self.a3, 0]
        ]

        # Calculate transformation matrices
        H_total = np.eye(4)
        for i in range(3):
            theta, alpha, r, d = DH[i]
            
            H = np.array([
                [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), r*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]
            ])
            
            H_total = np.dot(H_total, H)

        # Extract position and orientation
        position = H_total[:3, 3]
        orientation = quaternion_from_matrix(H_total)
        
        return position, orientation

    def publish_pose(self, pose):
        position, orientation = pose
        
        pose_msg = PoseStamped()
        #pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "r_sho_pitch"
        
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"End Effector Position: X: {position[0]:.1f}cm, Y: {position[1]:.1f}cm, Z: {position[2]:.1f}cm")

def main(args=None):
    rclpy.init(args=args)
    node = FKPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()