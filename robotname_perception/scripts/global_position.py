import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TransformListener(Node):
    def __init__(self):
        super().__init__('transform_listener')
        self.publisher = self.create_publisher(TransformStamped, 'global_position', 10)
        self.tf_buffer = tf2_ros.Buffer(self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def publish_transform_as_msg(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.LookupException as e:
            self.get_logger().warn(str(e))
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(str(e))
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(str(e))
            return

        # Create a TransformStamped message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = 'base_link'
        transform_msg.transform = transform.transform

        # Publish the transform message
        self.publisher.publish(transform_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformListener()

    while rclpy.ok():
        node.publish_transform_as_msg()
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
