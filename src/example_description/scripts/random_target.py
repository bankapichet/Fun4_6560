#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from fun4_interfaces.srv import StartRandomTarget 
from sensor_msgs.msg import JointState

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

class RandomPosition(Node):
    def __init__(self):
        super().__init__('random_position_node')
        
        
        self.srv = self.create_service(StartRandomTarget, 'start_random_target', self.start_random_target_callback)

        self.position_pub = self.create_publisher(PoseStamped, '/target', 10)
        self.position2_pub = self.create_publisher(JointState, '/q_target', 10)

    def start_random_target_callback(self, request, response):
       
        self.random_joint_angles_and_calculate_fk()
        response.success = True  
        return response

    def random_joint_angles_and_calculate_fk(self):
        
        q1 = np.random.uniform(-np.pi, np.pi)
        q2 = np.random.uniform(-np.pi, np.pi)
        q3 = np.random.uniform(-np.pi, np.pi)

        joint_angles = [q1, q2, q3]

        
        end_effector_pos = forward_kinematics(joint_angles)

       
        if self.is_within_workspace(end_effector_pos):
            print("inworkspace")
           
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'link_0'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = float(end_effector_pos[0])
            target_pose.pose.position.y = float(end_effector_pos[1])
            target_pose.pose.position.z = float(end_effector_pos[2])

          
            self.position_pub.publish(target_pose)

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3']
            joint_state_msg.position = joint_angles

          
            self.position2_pub.publish(joint_state_msg)
        else:
            print("notinworkspace")

    def is_within_workspace(self, end_effector_pos):
        x, y, z = end_effector_pos
        within_x = WORKSPACE_LIMITS['x'][0] <= x <= WORKSPACE_LIMITS['x'][1]
        within_y = WORKSPACE_LIMITS['y'][0] <= y <= WORKSPACE_LIMITS['y'][1]
        within_z = WORKSPACE_LIMITS['z'][0] <= z <= WORKSPACE_LIMITS['z'][1]
    
        return within_x and within_y and within_z


def main(args=None):
    rclpy.init(args=args)
    node = RandomPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
