#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

WORKSPACE_LIMITS = {
    'x': (-0.53, 0.53), 
    'y': (-0.53, 0.53),
    'z': (-0.33, 0.73)  
}

def forward_kinematics(joint_angles):
    q1, q2, q3 = joint_angles  

    
    A0 = np.array([
        [np.cos(q1), -np.sin(q1), 0, 0],
        [np.sin(q1), np.cos(q1), 0, 0],
        [0, 0, 1, 0.2],
        [0, 0, 0, 1]
    ])
    A1 = np.array([
        [1, 0, 0, 0],
        [0, np.cos(-np.pi/2),  -np.sin(-np.pi/2), 0],
        [0, np.sin(-np.pi/2),  np.cos(-np.pi/2), 0],
        [0, 0,  0, 1]
    ])
    A2 = np.array([
        [np.cos(q2), -np.sin(q2), 0, 0],
        [np.sin(q2), np.cos(q2), 0, 0],
        [0, 0, 1, -0.12],
        [0, 0, 0, 1]
    ])
    A3 = np.array([
        [1, 0, 0, 0],
        [0, np.cos(np.pi/2),  -np.sin(np.pi/2), 0],
        [0, np.sin(np.pi/2),  np.cos(np.pi/2), 0],
        [0, 0,  0, 1]
    ])
    A4 = np.array([
        [1, 0, 0, 0],
        [0, 1,  0, 0],
        [0, 0, 1, 0.25],
        [0, 0,  0, 1]
    ])
    A5 = np.array([
        [1, 0, 0, 0],
        [0, np.cos(-np.pi/2),  -np.sin(-np.pi/2), 0],
        [0, np.sin(-np.pi/2),  np.cos(-np.pi/2), 0],
        [0, 0,  0, 1]
    ])
    A6 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.1],
        [0, 0, 0, 1]
    ])
    A7 = np.array([
        [np.cos(q3), -np.sin(q3), 0, 0],
        [np.sin(q3), np.cos(q3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    A8 = np.array([
        [1, 0, 0, 0],
        [0, np.cos(np.pi/2),  -np.sin(np.pi/2), 0],
        [0, np.sin(np.pi/2),  np.cos(np.pi/2), 0],
        [0, 0,  0, 1]
    ])
    A9 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.28],
        [0, 0, 0, 1]
    ])

   
    A_final = A0 @ A1 @ A2 @ A3 @ A4 @ A5 @ A6 @ A7 @ A8 @ A9

    
    end_effector_pos = A_final[0:3, 3]
    return end_effector_pos

class WorkspaceNode(Node):
    def __init__(self):
        super().__init__('workspace_node')

       
        self.timer = self.create_timer(1.0, self.timer_callback)

       
        self.num_samples = 30 
        
    def calculate_workspace(self):
        
        joint1_samples = np.linspace(-np.pi, np.pi, self.num_samples)
        joint2_samples = np.linspace(-np.pi, np.pi, self.num_samples)
        joint3_samples = np.linspace(-np.pi, np.pi, self.num_samples)

        workspace_points = []

        
        for theta1 in joint1_samples:
            for theta2 in joint2_samples:
                for theta3 in joint3_samples:
                    joint_angles = [theta1, theta2, theta3]  
                    end_effector_pos = forward_kinematics(joint_angles)
                    workspace_points.append(end_effector_pos)

        return np.array(workspace_points)
 
    def plot_workspace(self, workspace_points):
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(
            workspace_points[:, 0],
            workspace_points[:, 1],
            workspace_points[:, 2],
            c='green', marker='o', s=1)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Robot Arm Workspace')
        plt.show()

    def timer_callback(self):
     
        self.get_logger().info('Calculating workspace...')
        workspace_points = self.calculate_workspace()
        self.plot_workspace(workspace_points)

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
