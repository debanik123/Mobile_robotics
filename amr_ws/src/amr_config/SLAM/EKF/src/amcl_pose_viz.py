import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt

class AMCLPosePlotter(Node):
    def __init__(self):
        super().__init__('amcl_pose_plotter')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.pose_x = []
        self.pose_y = []

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pose_x.append(x)
        self.pose_y.append(y)
        self.plot_pose()

    def plot_pose(self):
        plt.plot(self.pose_x, self.pose_y, 'ro-')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('AMCL Pose Plot')
        plt.pause(0.1)
        plt.clf()

def main(args=None):
    rclpy.init(args=args)
    plotter = AMCLPosePlotter()
    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
