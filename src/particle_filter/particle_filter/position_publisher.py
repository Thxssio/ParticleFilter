#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('pose_pub')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_model_state_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        while not self.get_model_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /gazebo/get_model_state not available, waiting...')
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.header = Header()
        self.header.frame_id = 'odom'

        self.model_request = GetModelState.Request()
        self.model_request.model_name = 'triton'

    def timer_callback(self):
        future = self.get_model_state_client.call_async(self.model_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            odom = Odometry()
            odom.pose.pose = result.pose
            odom.twist.twist = result.twist
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'

            self.odom_pub.publish(odom)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = result.pose.position.x
            t.transform.translation.y = result.pose.position.y
            t.transform.translation.z = result.pose.position.z
            t.transform.rotation = result.pose.orientation

            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
