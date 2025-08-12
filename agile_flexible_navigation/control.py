import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList


class ViconSubscriber(Node):
    """Constants"""

    def __init__(self):
        super().__init__('vicon_subscriber')
        self.subscription = self.create_subscription(
            PositionList,
            "vicon/default/data",
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        for i in range(msg.n):
            self.get_logger().info('subject "%s" with segment %s:' %(msg.positions[i].subject_name, msg.positions[i].segment_name))
            self.get_logger().info('I heard translation in x, y, z: "%f", "%f", "%f"' % (msg.positions[i].x_trans, msg.positions[i].y_trans, msg.positions[i].z_trans))
            self.get_logger().info('I heard rotation in x, y, z, w: "%f", "%f", "%f", "%f": ' % (msg.positions[i].x_rot, msg.positions[i].y_rot, msg.positions[i].z_rot, msg.positions[i].w))


def main(args=None):
    rclpy.init(args=args)

    vicon_subscriber = ViconSubscriber()

    rclpy.spin(vicon_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vicon_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
