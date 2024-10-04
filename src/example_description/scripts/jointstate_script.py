#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi
from spatialmath import SE3
import numpy as np
import roboticstoolbox as rtb
from tf2_ros import TransformListener, Buffer, TransformException
from fun4_interfaces.srv import Controlmode
from geometry_msgs.msg import TransformStamped ,PoseStamped
from std_msgs.msg import Float64

import time
class Control(Node):
    def __init__(self):
        super().__init__('control_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

       

        self.diff_pub=self.create_publisher(PoseStamped,"/Diff_position",10)

        self.target_frame = 'end_effector'
        self.source_frame = 'link_0'
        self.dt = 0.01
        self.create_timer(self.dt, self.sim_loop)
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.name = ["joint_1", "joint_2", "joint_3"]
       
        
        self.get_logger().info('Mode service is ready to receive requests.')
        self.input_sub = self.create_subscription(JointState, "/input_states", self.input_state_callback, 10)
        
        
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(a=0, alpha=0, d=0.2, offset=0),         
                rtb.RevoluteMDH(a=0, alpha=-pi/2, d=0, offset=-pi/2),    
                rtb.RevoluteMDH(a=0.25, alpha=0, d=-0.02, offset=pi/2),  
            ],
            tool=SE3.Rx(pi/2) @ SE3.Tz(0.28),
            name="RRR_Robot"
        )

        self.q = [0.0, 0.3, 0.9]
        self.q2 = []  
        self.q_sol = [0.0, 0.0, 0.0] 
        self.q_now = [0.0, 0.0, 0.0]  
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.name = ["joint_1", "joint_2", "joint_3"]  
        self.pose_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.ik_solution_q = None
        
        
        self.previous_position = None



    def inverse_kinematics(self, Xe, Ye, Ze):
        T_desired = SE3(Xe, Ye, Ze)
        IKSolution = self.robot.ikine_LM(T_desired, mask = [1,1,1,0,0,0])
       
        

    def input_state_callback(self, msg):
        if len(msg.position) >= 3:
          
            self.q_sol = list(msg.position[:3])  
           
        else:
            self.get_logger().error("Received input state has fewer than 3 joint angles.")



    def on_timer(self):
        from_frame_rel = 'link_0'
        to_frame_rel = 'end_effector'  

        if not self.tf_buffer.can_transform(from_frame_rel, to_frame_rel, rclpy.time.Time()):
            self.get_logger().info(f'ยังไม่พบ TF จาก {from_frame_rel} ไป {to_frame_rel}')
            return

        try:
          
            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())  
        except TransformException as ex:
            self.get_logger().info(f'ไม่สามารถ transform จาก {from_frame_rel} ไป {to_frame_rel}: {ex}')
            return

      
        end_effector_pos = t.transform.translation
        current_position = [end_effector_pos.x, end_effector_pos.y, end_effector_pos.z]
        # self.get_logger().info(f" end_effector positions: {current_position}")
        self.Xe = end_effector_pos.x
        self.Ye = end_effector_pos.y
        self.Ze = end_effector_pos.z
        end_effector_pos = PoseStamped()
        end_effector_pos.header.frame_id = 'link_0'
        end_effector_pos.header.stamp = self.get_clock().now().to_msg()
        end_effector_pos.pose.position.x = current_position[0]
        end_effector_pos.pose.position.y = current_position[1]
        end_effector_pos.pose.position.z = current_position[2]
        self.pose_pub.publish( end_effector_pos)
        if self.previous_position is None or self.position_changed(current_position):
            self.previous_position = current_position
         
            
            
            self.inverse_kinematics(self.Xe, self.Ye, self.Ze)

            

   
    def sim_loop(self):
        
        tolerance = 0.01 
        q_d = [0.9, 0.9, 0.9]  
        diff_msg = PoseStamped()
        diff_msg.header.frame_id = "base_link"  
        diff_msg.header.stamp = self.get_clock().now().to_msg()

      

      
        for i in range(len(self.q_now)):
          
            if abs(self.q_sol[i] - self.q_now[i]) > tolerance:
               
                direction = np.sign(self.q_sol[i] - self.q_now[i]) 
                self.q_now[i] += direction * q_d[i] * self.dt

           
            if abs(self.q_sol[i] - self.q_now[i]) <= tolerance:
                self.q_now[i] = self.q_sol[i]
            print(abs(self.q_sol[i] - self.q_now[i]) )
            diff_value = abs(self.q_sol[i] - self.q_now[i])

       
            self.get_logger().info(f"Joint {i+1} difference: {diff_value}")

        
            if i == 0:
                diff_msg.pose.position.x = diff_value  
            elif i == 1:
                diff_msg.pose.position.y = diff_value  
            elif i == 2:
                diff_msg.pose.position.z = diff_value  

        
            self.diff_pub.publish(diff_msg)
                

            
            self.publish_joint_state()
       
      
        

        
    def publish_joint_state(self):
       
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.name

        
        msg.position = list(map(float, self.q_now))  

        
        self.joint_pub.publish(msg)
        
    def position_changed(self, current_position):
        tolerance = 0.005
        for i in range(3):
            if abs(current_position[i] - self.previous_position[i]) > tolerance:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()