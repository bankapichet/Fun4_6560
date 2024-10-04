#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist, Pose
import numpy as np
from fun4_interfaces.srv import Controlmode, StartRandomTarget
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(a=0, alpha=0, d=0.2, offset=0),
                rtb.RevoluteMDH(a=0, alpha=-pi/2, d=0, offset=-pi/2),
                rtb.RevoluteMDH(a=0.25, alpha=0, d=-0.02, offset=pi/2),
            ],
            tool=SE3.Rx(pi/2) @ SE3.Tz(0.28),
            name="RRR_Robot"
        )

       
        self.random_target_client = self.create_client(StartRandomTarget, 'start_random_target')

        
        self.mode_service = self.create_service(Controlmode, 'change_mode', self.change_mode_callback)
        
        
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.input_pub = self.create_publisher(JointState, "/input_states", 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.teleop_callback, 10)
        self.q_target_sub = self.create_subscription(JointState, '/q_target', self.q_target_callback, 10)
        
       
        self.diff_position_sub = self.create_subscription(PoseStamped, '/Diff_position', self.diff_position_callback, 10)

        self.current_mode = "Idle"
        self.q = [0.0, 0.0, 0.0]  
        self.q_target = None  
        self.diff_position = [None, None, None]  
        self.name = ["joint_1", "joint_2", "joint_3"]

        
        self.was_nonzero = False

        self.get_logger().info("Robot Controller Node Started")
        self.timer = self.create_timer(0.1, self.update)

    def q_target_callback(self, msg):
        if len(msg.position) >= 3:
            self.q_target = list(msg.position[:3])
        else:
            self.get_logger().warn("Received q_target has fewer than 3 joint angles.")

    def diff_position_callback(self, msg):
       
        self.diff_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

        
        if any(val != 0.00000 for val in self.diff_position):
            self.was_nonzero = True  

    def change_mode_callback(self, request, response):
        self.get_logger().info(f"Request to change mode to: {request.mode}")

        if request.mode == "IPK":
            self.current_mode = "IPK"
            Xe, Ye, Ze = request.x, request.y, request.z
            self.get_logger().info(f"Target Position - X: {Xe}, Y: {Ye}, Z: {Ze}")
            success, solution = self.inverse_pose_kinematics(Xe, Ye, Ze)

            if success:
                response.success = True
                response.message = f"Mode changed to {request.mode} with successful IK at X: {Xe}, Y: {Ye}, Z: {Ze}"
            else:
                response.success = False
                response.message = f"Mode changed to {request.mode}, but no IK solution found."

            self.get_logger().info(response.message)

        elif request.mode == "Auto":
            self.current_mode = "Auto"
            response.success = True
            response.message = f"Mode changed to {request.mode}"
            self.get_logger().info(response.message)

        elif request.mode == "Teleop":
            self.current_mode = "Teleop"
            response.success = True
            response.message = f"Mode changed to {request.mode}"
            self.get_logger().info(response.message)

        else:
            response.success = False
            response.message = "Invalid mode requested"
            self.get_logger().error(response.message)

        return response

    def inverse_pose_kinematics(self, Xe, Ye, Ze):
        self.get_logger().info("Inverse Pose Kinematics (IPK) Mode Active")
        T_desired = SE3(Xe, Ye, Ze)

        try:
            ik_solution = self.robot.ikine_LM(T_desired, mask=[1, 1, 1, 0, 0, 0])

            if ik_solution.success:
                self.get_logger().info(f"Solution For Target: {ik_solution.q}")
                self.q = ik_solution.q  
                self.publish_joint_states()
                self.publish_joint_state(self.q)
                return True, ik_solution.q
            else:
                self.get_logger().warn("No IK Solution found.")
                return False, None
        except Exception as e:
            self.get_logger().error(f"Error during IK computation: {str(e)}")
            return False, None

    def publish_joint_state(self, joint_angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.name
        msg.position = list(map(float, joint_angles))
        self.input_pub.publish(msg)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.name
        msg.position = [float(i) for i in self.q]
        self.joint_pub.publish(msg)

    def teleop_callback(self, msg):
        if self.current_mode != "Teleop":
            return
        self.get_logger().info("Teleoperation Mode Active")
        

    def autonomous_mode(self):
        self.get_logger().info("Autonomous Mode Active")
        if self.q_target is None:
            self.get_logger().warn("No q_target received yet, requesting random target.")
            self.request_random_target()
            return
        self.publish_joint_state(self.q_target)
        self.get_logger().info(f"Published autonomous q_target to /input_states: {self.q_target}")
        self.get_logger().info(f"diff_q :  {self.diff_position}")
        
    def request_random_target(self):
      
        if not self.random_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service start_random_target not available, waiting...')
            return

       
        request = StartRandomTarget.Request()
        future = self.random_target_client.call_async(request)
        future.add_done_callback(self.random_target_callback)

    def random_target_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.q_target = [response.random_target.x, response.random_target.y, response.random_target.z]
                self.get_logger().info(f"Random target received: {self.q_target}")
            else:
                self.get_logger().warn("Failed to get random target.")
        except Exception as e:
            self.get_logger().error(f"Error in random target request: {str(e)}")

    def update(self):
        if self.current_mode == "Auto":
           
            if self.diff_position == [0.00000, 0.00000, 0.00000] and self.was_nonzero:
                self.get_logger().info("All Diff Position values are 0 again, requesting new random target.")
                self.request_random_target()
                self.was_nonzero = False 
            else:
                self.autonomous_mode()

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Robot Controller")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

