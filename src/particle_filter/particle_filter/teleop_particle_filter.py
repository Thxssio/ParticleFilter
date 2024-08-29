#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Thread
import sys
import time
from pynput import keyboard

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_robot')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 2)
        self.vel_msg = Twist()
        self.key_state = {}
        self.LIN_SPEED = 0.2
        self.ANG_SPEED = 1.0
        self.rate = self.create_rate(10)

    def key_update(self, key, state):
        if key not in self.key_state:
            self.key_state[key] = state
            return True
        if state != self.key_state[key]:
            self.key_state[key] = state
            return True
        return False

    def key_press(self, key):
        if key == keyboard.Key.esc:
            self.get_logger().info('\nPress Ctrl+C to exit')
            rclpy.shutdown()
            return False
        try:
            k = key.char
        except:
            k = key.name

        change = self.key_update(key, True)
        if change:
            if k in ['w', 'up']:
                self.vel_msg.linear.x += self.LIN_SPEED
            elif k in ['s', 'down']:
                self.vel_msg.linear.x -= self.LIN_SPEED
            elif k in ['d', 'right']:
                self.vel_msg.linear.y -= self.LIN_SPEED
            elif k in ['a', 'left']:
                self.vel_msg.linear.y += self.LIN_SPEED
            elif k in ['e']:
                self.vel_msg.angular.z -= self.ANG_SPEED
            elif k in ['q']:
                self.vel_msg.angular.z += self.ANG_SPEED
            elif k in ['x']:
                self.LIN_SPEED += 0.1
            elif k in ['z']:
                self.LIN_SPEED -= 0.1
        return True

    def key_release(self, key):
        try:
            k = key.char
        except:
            k = key.name

        change = self.key_update(key, False)
        if change:
            if k in ['w', 'up']:
                self.vel_msg.linear.x = 0
            elif k in ['s', 'down']:
                self.vel_msg.linear.x = 0
            elif k in ['d', 'right']:
                self.vel_msg.linear.y = 0
            elif k in ['a', 'left']:
                self.vel_msg.linear.y = 0
            elif k in ['e']:
                self.vel_msg.angular.z = 0
            elif k in ['q']:
                self.vel_msg.angular.z = 0
        return True

    def user_display(self):
        self.get_logger().info('Use WSAD or the ARROW KEYS to control Triton.\nUse Q & E to rotate Triton.\nUse x/z to increase/decrease speed')
        while rclpy.ok():
            log_str = f"\r\t\tX: {self.vel_msg.linear.x}\tY: {self.vel_msg.linear.y}\tTHETA: {self.vel_msg.angular.z}\t"
            sys.stdout.write(log_str)
            sys.stdout.flush()
            self.vel_pub.publish(self.vel_msg)
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    key_listener = keyboard.Listener(on_press=node.key_press, on_release=node.key_release) 
    key_listener.start()

    display_thread = Thread(target=node.user_display)
    display_thread.start()

    rclpy.spin(node)

    key_listener.join()
    display_thread.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
