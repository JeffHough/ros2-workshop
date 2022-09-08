#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

  def __init__(self):
    super().__init__('minimal_subscriber')
    self.create_subscription(String,"/mae_tutorials/topic",self.listener_callback,1)

  def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
  rclpy.init(args=args)
  minimal_subscriber = MinimalSubscriber()

  # NOTE - THIS IS STILL A BLOCKING CALL! Just we aren't doing anything else,
  # so that is fine.
  rclpy.spin(minimal_subscriber)

  # Destroy the node:
  minimal_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()