#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.5 
        self.turn = 1.0   
        self.move_bindings = {
            'a': (0.5, 0, 0),   
            's': (0, 0.5, 0),  
            'd': (0, 0, 0.5),   
            'f': (-0.5, 0, 0),  
            'g': (0, -0.5, 0),  
            'h': (0, 0, -0.5)   
        }
        self.speed_bindings = {
            'w': (1.1, 1.0), 
            'x': (0.9, 1.0),   
            'e': (1.0, 1.1),   
            'c': (1.0, 0.9),   
            'z': (0.9, 0.9)    
        }

    def get_key(self):
      
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key
    def run(self):
        print("Use keys to control the robot:")
        print("w/x : Increase/Decrease linear speed")
        print("e/c : Increase/Decrease angular speed")
        print("a/s/d : Move 0.5 once in x/y/z")
        print("f/g/h : Move -0.5 once in x/y/z")
        print("z : Decrease both linear and angular speed")
        print("CTRL-C to quit")

        while True:
            key = self.get_key()

            if key in self.move_bindings:
               
                twist = Twist()
                twist.linear.x = float(self.move_bindings[key][0]) 
                twist.linear.y = float(self.move_bindings[key][1]) 
                twist.linear.z = float(self.move_bindings[key][2])  
                self.cmd_vel_pub.publish(twist)

               
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                self.cmd_vel_pub.publish(twist)

            elif key in self.speed_bindings:
                
                self.speed *= self.speed_bindings[key][0]
                self.turn *= self.speed_bindings[key][1]

                print(f"Speed: {self.speed:.2f}, Turn: {self.turn:.2f}")

            elif key == '\x03':  
                break
            else:
               
                twist = Twist()
                twist.linear.x = float(self.speed if key == 'w' else -self.speed if key == 'x' else 0.0)
                twist.angular.z = float(self.turn if key == 'e' else -self.turn if key == 'c' else 0.0)
                self.cmd_vel_pub.publish(twist)

        
        twist = Twist()
        self.cmd_vel_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
