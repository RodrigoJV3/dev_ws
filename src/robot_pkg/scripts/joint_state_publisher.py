#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DummyJointStatePublisher(Node):
    def __init__(self):
        super().__init__('dummy_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.dummy_joint_states = JointState()
        
        # Update these names to match your actual joint names
        self.dummy_joint_states.name = ['U_base_wheel_1_joint', 'U_wheel_1_joint', 'U_base_wheel_2_joint', 'U_wheel_2_joint']
        
        # Provide dummy values for each joint
        self.dummy_joint_states.position = [0.0, 0.0, 0.0, 0.0]  # Replace with any dummy values

    def publish_joint_states(self):
        # Update the timestamp
        self.dummy_joint_states.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.dummy_joint_states)

def main(args=None):
    rclpy.init(args=args)
    node = DummyJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#chmod +x ~/dev_ws/src/robot_pkg/scripts/joint_state_publisher.py
