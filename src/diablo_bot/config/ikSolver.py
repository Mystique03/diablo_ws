#!/usr/bin/env python3
# Program to calculate IK of right arm, and publish to the joint_trajectory

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from math import sqrt, atan, acos, pi

np.set_printoptions(precision=6, suppress=True)

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.subscription = self.create_subscription(
            Point, '/desired_end_effector_position', self.ik_callback, 10)
        self.publisher = self.create_publisher(
            JointTrajectory, '/right_arm_controller/joint_trajectory', 10)

        self.initial_positions = {
            "r_sho_pitch": 0.0,
            "r_sho_roll": -1.25,
            "r_el": 0.0,
            "r_wrist": 0.0
        }
        #link leghths in m
        
        self.a1 = 5.16 
        self.a2 = 16.31
        self.a3 = 19.8 

        self.joint_limits ={
            "r_sho_pitch" : (-1.9635, 1.9635),
            "r_sho_roll" : (-1.600, -0.900),
            "r_el" : (-1.570, 0.0),
            "r_wrist" : (-1.5708, 1.5708)

        }

    def clamp_angle(self, joint_name, angle):
        lower, upper = self.joint_limits[joint_name]
        return np.clip(angle, lower, upper)
    
        
    def reset_to_ip(self):
        self.get_logger().info("Invalide position. Resetting arm to initial position")
        self.publish_joint(
            self.clamp_angle("r_sho_pitch", self.initial_positions["r_sho_pitch"]),
            self.clamp_angle("r_sho_roll", self.initial_positions["r_sho_roll"]),
            self.clamp_angle("r_el", self.initial_positions["r_el"]),
            self.clamp_angle("r_wrist", self.initial_positions["r_wrist"])
        )

    def ik_callback(self, msg):
        # Desired end-effector position
        x = msg.x
        y = msg.y
        z = msg.z


        try: 
        # IK calculations
            r1 = sqrt(x**2 + y**2)  # Eq.1
            
            acos1 = (self.a3**2 - self.a2**2 - r1**2)/(-2*self.a2*r1)
            acos1 = np.clip(acos1, -1.0, 1.0)
            phi1 = acos(acos1)  # Eq.2

            phi2 = atan(x/y)  # Eq.3

            theta1 = self.clamp_angle("r_sho_pitch", phi1 - phi2)  # Eq.4

            acos2= ((r1**2 - self.a2**2 - self.a3**2)/(-2*self.a2*self.a3))
            acos2 = np.clip(acos2, -1.0, 1.0)
            phi3 = acos(acos2)   #Eq.5
        
            theta3 = self.clamp_angle("r_el", pi - phi3)  # Eq.6

            r2 = z - self.a1  # Eq.7
            r3 = sqrt(x**2 + y**2)  # Eq.8
            theta2 = self.clamp_angle("r_sho_roll", atan(r2/r3))  # Eq.9


            theta4 = self.clamp_angle("r_wrist" , 0.0)

            self.get_logger().info(
                    f"Joint angles: "
                    f"r_sho_pitch={theta1:.3f} rad, "
                    f"r_sho_roll={theta2:.3f} rad, "
                    f"r_el={theta3:.3f} rad"
                )

        # Publish joint angles
            self.publish_joint(theta1, theta2, theta3, theta4)

        except (ZeroDivisionError, ValueError) as e:
            self.reset_to_ip()

    def publish_joint(self, theta1, theta2, theta3, theta4):
        trajectory = JointTrajectory()
        trajectory.joint_names = ["r_sho_pitch", "r_sho_roll", "r_el", "r_wrist"]
        point = JointTrajectoryPoint()
        point.positions = [theta1, theta2, theta3, theta4]
        point.time_from_start.sec = 2

        trajectory.points.append(point)
        self.publisher.publish(trajectory)
        

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()