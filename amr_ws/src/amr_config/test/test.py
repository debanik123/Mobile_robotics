import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        frame1 = "map"  # Replace with your desired frame ID
        frame2 = "base_footprint"  # Replace with your desired frame ID
        
        
        for transform in msg.transforms:
            print(transform.header.frame_id, transform.child_frame_id)
            # if transform.header.frame_id == frame1 and transform.child_frame_id == frame2:
            #     print(transform.header.frame_id, transform.child_frame_id)
            #     translation = transform.transform.translation
            #     rotation = transform.transform.rotation
            #     self.get_logger().info(
            #         f"Transformation from {frame1} to {frame2}: Translation: [{translation.x}, {translation.y}, {translation.z}], Rotation: [{rotation.x}, {rotation.y}, {rotation.z}, {rotation.w}]"
            #     )
            #     break
        print("------------------------------------------------------\
              --------------------------------------------------------")

def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)
    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
