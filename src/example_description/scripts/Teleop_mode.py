#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi
from spatialmath import SE3
import numpy as np
import roboticstoolbox as rtb
from tf2_ros import TransformListener, Buffer, TransformException
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from fun4_interfaces.srv import Controlmode  

class ModeServiceNode(Node):
    def __init__(self):
        super().__init__('mode_service_node')
        self.ref_mode = 0  
        self.current_twist = Twist()  
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.lookup_transform_callback)
        self.dt = 0.01
        self.create_timer(self.dt, self.sim_loop)
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.name = ["joint_1", "joint_2", "joint_3"]

        self.singularity_pub = self.create_publisher(String, '/singularity_warning', 10)

        self.mode = 0 
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.teleop_callback, 10)

       
        self.ref_mode_sub = self.create_subscription(Int32, '/ref_mode', self.ref_mode_callback, 10)

       
        self.mode_service = self.create_service(Controlmode, 'change_mode', self.change_mode_callback)

        self.ref = None
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(a=0, alpha=0, d=0.2, offset=0),
                rtb.RevoluteMDH(a=0, alpha=-pi/2, d=0, offset=-pi/2),
                rtb.RevoluteMDH(a=0.25, alpha=0, d=-0.02, offset=pi/2),
            ],
            tool=SE3.Rx(pi/2) @ SE3.Tz(0.28),
            name="RRR_Robot"
        )

        self.q_sol = [0.0, 0.0, 0.0]
        self.q_now = [0.0, 0.0, 0.0]
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

    def ref_mode_callback(self, msg):
       
        self.ref_mode = msg.data 
        self.get_logger().info(f"Received ref_mode: {self.ref_mode}")

    def change_mode_callback(self, request, response):
        
        if request.mode == "Teleop":
            self.ref_mode = request.x 
            self.get_logger().info(f"Changed to Teleop mode with ref_mode: {self.ref_mode}")
            response.success = True
            response.message = f"Changed mode to Teleop with ref_mode {self.ref_mode}"
        else:
            response.success = False
            response.message = "Unsupported mode"
        return response

    def detect_singularity(self, J):
        condition_number = np.linalg.cond(J)
        singularity_threshold = 1000  

        if condition_number > singularity_threshold:
            self.get_logger().warn("Near singularity detected!")
            warning_msg = String()
            warning_msg.data = "Singularity detected. Robot will stop."
            self.singularity_pub.publish(warning_msg)
            return True
        return False

    def teleop_callback(self, msg):
      
        self.current_twist = msg 
        v_linear = msg.linear  
        v_angular = msg.angular 

      
        if self.ref_mode == 1:
            self.move_with_base(v_linear, v_angular)
        elif self.ref_mode == 0:
            self.move_with_end_effector(v_linear, v_angular)

    def move_with_end_effector(self, v_linear, v_angular):
       
        
        v = np.array([v_linear.x, v_linear.y, v_linear.z])

       
        J = self.robot.jacob0(self.q_now)  

     
        J_linear = J[0:3, :]

      
        q_dot = np.linalg.pinv(J_linear) @ v

       
        q_dot_scaled = self.velocity_scaling(self.q_now, q_dot)
        q_new = self.q_now + q_dot_scaled * self.dt
        q_new = self.normalize_joint_angles(q_new)

      
        if self.check_joint_limits(q_new):
            self.q_now = q_new

      
        self.publish_joint_state()

    def move_with_base(self, v_linear, v_angular):
        
        v = np.hstack((v_linear.x, v_linear.y, v_linear.z, v_angular.x, v_angular.y, v_angular.z))

       
        J = self.robot.jacob0(self.q_now)  

      
        q_dot = np.linalg.pinv(J) @ v

      
        q_dot_scaled = self.velocity_scaling(self.q_now, q_dot)
        q_new = self.q_now + q_dot_scaled * self.dt
        q_new = self.normalize_joint_angles(q_new)

        if self.check_joint_limits(q_new):
            self.q_now = q_new

       
        self.publish_joint_state()

    def publish_joint_state(self):
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.name
        msg.position = list(map(float, self.q_now))
        self.joint_pub.publish(msg)

    def normalize_angle(self, angle):
       
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def normalize_joint_angles(self, q):
       
        return [self.normalize_angle(angle) for angle in q]

    def check_joint_limits(self, q):
        
        for i, angle in enumerate(q):
            if not -pi <= angle <= pi:
                self.get_logger().warning(f"Joint {i+1} out of limit: {angle:.2f}.")
                return False
        return True

    def velocity_scaling(self, q, q_dot):
       
        scaled_q_dot = []
        for i, (angle, velocity) in enumerate(zip(q, q_dot)):
            distance_to_limit = min(abs(pi - angle), abs(-pi - angle))
            scaling_factor = max(0.1, distance_to_limit / pi)
            scaled_q_dot.append(velocity * scaling_factor)
        return np.array(scaled_q_dot)

    def lookup_transform_callback(self):
      
        try:
            now = rclpy.time.Time(seconds=0)
            transform = self.tf_buffer.lookup_transform('link_0', 'end_effector', now)
            self.process_transform(transform)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform link_0 to end_effector: {ex}')

    def process_transform(self, transform):
        
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        matrix = SE3(tx, ty, tz)

        self.q_sol, *_ = self.robot.ikine_LM(matrix, q0=self.q_now, tol=1e-6)

    def sim_loop(self):
        
        v_linear = self.current_twist.linear
        v_angular = self.current_twist.angular

        if self.ref_mode == 1:
            J = self.robot.jacob0(self.q_now)
            J_linear = J[0:3, :]

            if self.detect_singularity(J_linear):
                self.q_now = self.q_now
                self.publish_joint_state()
                return

            v = np.array([v_linear.x, v_linear.y, v_linear.z])
            q_dot = np.linalg.pinv(J_linear) @ v
        elif self.ref_mode == 0:
            J_e = self.robot.jacobe(self.q_now)
            J_linear = J_e[0:3, :]

            if self.detect_singularity(J_linear):
                self.q_now = self.q_now
                self.publish_joint_state()
                return

            v = np.array([v_linear.x, v_linear.y, v_linear.z])
            q_dot = np.linalg.pinv(J_linear) @ v

        q_dot_scaled = self.velocity_scaling(self.q_now, q_dot)

        q_new = self.q_now + q_dot_scaled * self.dt
        q_new = self.normalize_joint_angles(q_new)

        if self.check_joint_limits(q_new):
            self.q_now = q_new

        self.publish_joint_state()

def main(args=None):
    rclpy.init(args=args)
    node = ModeServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
