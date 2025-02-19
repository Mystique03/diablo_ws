#!/usr/bin/env python3

# run to use joint_state_publisher_gui with gazebo

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateToTrajectory(Node):
    def __init__(self):
        super().__init__('joint_state_to_trajectory')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/right_arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(
            JointTrajectory, '/right_gripper_controller/joint_trajectory', 10)
    
    def joint_state_callback(self, msg):
        # Map joint states to arm controller
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = ['r_sho_pitch', 'r_sho_roll', 'r_el', 'r_wrist']
        arm_point = JointTrajectoryPoint()
        arm_point.positions = [msg.position[msg.name.index(j)] for j in arm_trajectory.joint_names]
        arm_trajectory.points.append(arm_point)
        self.arm_publisher.publish(arm_trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()