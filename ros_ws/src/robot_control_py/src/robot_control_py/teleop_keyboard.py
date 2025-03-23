#!/usr/bin/env python3

import sys
import math
import select
import termios
import tty
from threading import Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max linear speed by 10%
w/x : increase/decrease max angular speed by 10%
s : stop
b : toggle debug output

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
}

speedBindings = {
    'q': (1.1, 1),
    'z': (0.9, 1),
    'w': (1, 1.1),
    'x': (1, 0.9),
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )
        
        self.speed = 0.2
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0
        self.lock = Lock()
        
        self.debug = False
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Teleop Keyboard Node Started')
        self.get_logger().info(msg)
        
    def timer_callback(self):
        with self.lock:
            twist_msg = TwistStamped()
            twist_msg.header = Header()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            
            twist_msg.twist.linear.x = self.x * self.speed
            twist_msg.twist.linear.y = self.y * self.speed
            twist_msg.twist.linear.z = self.z * self.speed
            
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = self.th * self.turn
            
            self.publisher.publish(twist_msg)
            
            if self.debug:
                self.get_logger().info(f"Publishing: x={twist_msg.twist.linear.x:.2f}, z={twist_msg.twist.angular.z:.2f}")


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    teleop_node = KeyboardTeleop()
    
    speed = teleop_node.speed
    turn = teleop_node.turn
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(teleop_node)
    
    # Start the executor in a separate thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        while True:
            key = getKey(settings)
            if key == '\x03':  # CTRL-C
                break
                
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                
                with teleop_node.lock:
                    teleop_node.x = x
                    teleop_node.y = y
                    teleop_node.z = z
                    teleop_node.th = th
                    
            elif key in speedBindings.keys():
                with teleop_node.lock:
                    speed = speed * speedBindings[key][0]
                    turn = turn * speedBindings[key][1]
                    
                    teleop_node.speed = speed
                    teleop_node.turn = turn
                    
                    teleop_node.get_logger().info(f'Speed: {speed:.2f}, Turn: {turn:.2f}')
                    
            elif key == 's':
                with teleop_node.lock:
                    teleop_node.x = 0.0
                    teleop_node.y = 0.0
                    teleop_node.z = 0.0
                    teleop_node.th = 0.0
            
            elif key == 'b':
                with teleop_node.lock:
                    teleop_node.debug = not teleop_node.debug
                    teleop_node.get_logger().info(f'Debug mode: {"on" if teleop_node.debug else "off"}')
            
            else:
                # Skip updating if key not recognized
                if key == '\x1b':  # ESC
                    break
    
    except Exception as e:
        teleop_node.get_logger().error(f'Exception: {e}')
    
    finally:
        # Stop the robot
        twist_msg = TwistStamped()
        twist_msg.header = Header()
        twist_msg.header.stamp = teleop_node.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        teleop_node.publisher.publish(twist_msg)
        
        # Reset terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 