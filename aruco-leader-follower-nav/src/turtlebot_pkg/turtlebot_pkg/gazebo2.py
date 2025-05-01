#!/usr/bin/env python3
"""
Gazebo TF Broadcaster for Leader Robot (doc_robolab)
-----------------------------------------------------
This ROS 2 node subscribes to the simulated ground truth pose of the leader TurtleBot in Gazebo
and broadcasts a TF2 transformation from the world frame ("warehouse") to the robot frame
("doc_robolab/turtlebot4").

Main Features:
- Reads `/doc_robolab/sim_ground_truth_pose` (nav_msgs/Odometry)
- Publishes TF2 transform: `warehouse → doc_robolab/turtlebot4`
- Enables leader–follower simulation using TF tracking

Topics:
- Subscribed: `/doc_robolab/sim_ground_truth_pose`
- TF Published: `warehouse → doc_robolab/turtlebot4`

Authors: Armando Nicolella (LAM4R - University of Naples Federico II), Pasquale Stingo
"""



from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class FramePublisher2(Node):

    def __init__(self):
        # Initialize ROS node with name 'turtlebot'
        super().__init__('turtlebot')

        # Create TF2 broadcaster to send transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Set QoS profile: keep last 10 messages, best-effort
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,  
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        )

        # Subscribe to leader's ground truth pose in simulation
        self.subscription = self.create_subscription(
            Odometry,
            'doc_robolab/sim_ground_truth_pose',
            self.handle_turtle_pose,
            qos_profile=qos_profile)

    def handle_turtle_pose(self, msg):
        # Convert Odometry message into TF2 transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'warehouse'
        t.child_frame_id = 'doc_robolab/turtlebot4'

        # Set translation from pose
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Set orientation from pose
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Broadcast the TF2 transform
        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info('Transform Published:\n{}'.format(t))


def main():
    rclpy.init()
    node = FramePublisher2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
