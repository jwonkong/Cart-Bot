import time

from my_action_interface.action import Open

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32



class OpenActionServer(Node):

    def __init__(self):
        super().__init__('Open_action_server')
        self._action_server = ActionServer(
            self,
            Open,
            'open',
            self.execute_callback)
        self._open_publisher = self.create_publisher(
            Int32,
            'micro_ros_arduino_subscriber',
            QoSProfile(depth=10))

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        msg = Int32()
        msg.data = 1
        self._open_publisher.publish(msg) # 한번만 실행되나?
        # 조건문 해야하는데 일단 실행했음으로 succeed 반환
        goal_handle.succeed()
        result = Open.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    open_action_server = OpenActionServer()

    try:
        rclpy.spin(open_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()