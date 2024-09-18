import rclpy
import tf2_ros
# import math
# import sys
# import numpy as np
import tf2_msgs.msg
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
# from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        qos_profile = QoSProfile(depth=10)
        self.my_navigator = BasicNavigator()
        self.aruco_sub_node = self.create_subscription(ArucoMarkers, 'aruco_markers', self.aruco_sub_node_msg, qos_profile)
        self.my_node_pub = self.create_publisher(String, 'my_node_pub', qos_profile)
        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.create_subscription(
        #     tf2_msgs.msg.TFMessage,
        #     '/tf',
        #     self.tf_sub_callback,
        #     10
        # )

    def goal_pose_pub_node_msg(self, frame_ids, marker_ids):
        msg = String()
        try:
            tf_map_to_cart_base_0 = self.tf_buffer.lookup_transform(
                'map',
                'cart_base_0',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to cart_base_0: {ex}')
            return
        #tf_map_to_cart_base_0 =  self.tf_buffer.lookup_transform('marker_0', 'map', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=10.0))
        #tf_map_to_cart_base_0 =  self.tf_buffer.lookup_transform('marker_0', 'camera_color_optical_frame', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1.0))
        self.get_logger().info('{}'.format(tf_map_to_cart_base_0))
        my_goal_x = tf_map_to_cart_base_0.transform.translation.x
        my_goal_y = tf_map_to_cart_base_0.transform.translation.y

        my_goal_w = tf_map_to_cart_base_0.transform.rotation.w
        my_goal_z = tf_map_to_cart_base_0.transform.rotation.z

        my_goal_pose = PoseStamped()
        my_goal_pose.header.frame_id = 'map'
        my_goal_pose.header.stamp = self.my_navigator.get_clock().now().to_msg()
        my_goal_pose.pose.position.x = my_goal_x
        my_goal_pose.pose.position.y = my_goal_y
        my_goal_pose.pose.orientation.w = my_goal_w
        my_goal_pose.pose.orientation.z = my_goal_z
        self.my_navigator.goToPose(my_goal_pose)



        msg.data = 'frame_ids = ' + str(frame_ids) + '  marker_ids = ' + str(marker_ids[0])
        self.my_node_pub.publish(msg)

    def aruco_sub_node_msg(self, msg):
        self.get_logger().info('{}'.format(msg.header.frame_id))
        self.get_logger().info('{}'.format(msg.marker_ids))
        self.get_logger().info('{}'.format(msg.poses[0].position))
        self.get_logger().info('{}'.format(msg.poses[0].orientation))
        # 목표지점에 도달하기 전까지 아래함수가 한번만 실행되도록 해야함
        self.goal_pose_pub_node_msg(msg.header.frame_id, msg.marker_ids)

    # def tf_sub_callback(self, msg):
    #     for transform in msg.transforms:
    #         self.get_logger().info('Transform from:    {}'.format(transform.header.frame_id))
    #         self.get_logger().info('Transform to:     {}'.format(transform.child_frame_id))
    #         self.get_logger().info('Translation:    {}'.format(transform.transform.translation))
    #         self.get_logger().info('Rotation:     {}'.format(transform.transform.rotation))



def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        my_node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        my_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()