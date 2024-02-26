import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_covariance):
        # State vector [x, y, theta]
        self.x = np.array(initial_state, dtype=float)

        # Covariance matrix
        self.P = np.array(initial_covariance, dtype=float)

        # Process noise covariance matrix Q
        self.Q = np.diag([0.01, 0.01, 0.01])

        # Measurement noise covariance matrix R
        self.R = np.diag([0.1, 0.1])

    def predict(self, dt, linear_velocity, angular_velocity):
        # Predict state using odometry model
        self.x[0] += dt * linear_velocity * np.cos(self.x[2])
        self.x[1] += dt * linear_velocity * np.sin(self.x[2])
        self.x[2] += dt * angular_velocity

        # Compute the Jacobian of the process model
        F = np.array([
            [1, 0, -dt * linear_velocity * np.sin(self.x[2])],
            [0, 1, dt * linear_velocity * np.cos(self.x[2])],
            [0, 0, 1]
        ])

        # Update the covariance matrix
        self.P = np.dot(F, np.dot(self.P, F.T)) + self.Q

    def update(self, measurement):
        # Compute the Jacobian of the measurement model
        H = np.array([
            [np.cos(self.x[2]), -np.sin(self.x[2]), 0],
            [np.sin(self.x[2]), np.cos(self.x[2]), 0]
        ])
        # print(H.T.shape, measurement.shape)

        # Compute the innovation
        y = measurement - np.dot(H, self.x)
        # print(y)
        # print(np.dot(H, self.x).shape)
        # print(self.P.shape, H.T.shape, np.dot(H, np.dot(self.P, H.T)).shape, self.R.shape)
        # # Compute the innovation covariance
        S = np.dot(H, np.dot(self.P, H.T)) + self.R
        # print(S)
        # # Compute Kalman gain
        K = np.dot(self.P, np.dot(H.T, np.linalg.inv(S)))
        # print(K)
        # # Update state and covariance
        self.x = self.x+ np.dot(K, y)
        # print(self.x)
        self.P = self.P  - np.dot(K, np.dot(H, self.P))
        # print(self.P)

class EKFRos2Node(Node):
    def __init__(self):
        super().__init__('ekf_ros2_node')

        # Initialize EKF
        initial_state = [0.0, 0.0, 0.0]
        initial_covariance = np.diag([1.0, 1.0, 0.1])
        self.ekf = ExtendedKalmanFilter(initial_state, initial_covariance)
        self.ekf_last_update_time = 0.0
        # Subscribe to Lidar (scan) and odometry topics
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.estimated_trajectory_publisher = self.create_publisher(Odometry, 'estimated_trajectory', 10)

        # Initialize variables for plotting
        self.true_trajectory = []
        self.estimated_trajectory = []

    def lidar_callback(self, msg):
        # Process Lidar data here
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        for r, angle in zip(ranges, angles):
            if np.isfinite(r):
                measurement = np.array([r, angle])
                self.ekf.update(measurement)
        
        # print("ok")
        # Save the current state for plotting
        self.estimated_trajectory.append(self.ekf.x.copy())

    def odom_callback(self, msg):
        # Process odometry data here
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        dt = msg.header.stamp.sec - self.ekf_last_update_time
        self.ekf.predict(dt, linear_velocity, angular_velocity)

        # Save the current state for plotting
        self.true_trajectory.append(self.ekf.x.copy())
        # self.publish_estimated_trajectory()

        self.ekf_last_update_time = msg.header.stamp.sec
    
    def save_trajectory_data(self, true_filename='true_trajectory.csv', estimated_filename='estimated_trajectory.csv'):
        np.savetxt(true_filename, np.array(self.true_trajectory), delimiter=',')
        np.savetxt(estimated_filename, np.array(self.estimated_trajectory), delimiter=',')

    def publish_estimated_trajectory(self):
        if self.estimated_trajectory:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.ekf.x[0]
            odom_msg.pose.pose.position.y = self.ekf.x[1]
            odom_msg.pose.pose.orientation.z = np.sin(self.ekf.x[2] / 2.0)
            odom_msg.pose.pose.orientation.w = np.cos(self.ekf.x[2] / 2.0)

            self.estimated_trajectory_publisher.publish(odom_msg)
    
    def plot_trajectory(self):
        true_trajectory = np.array(self.true_trajectory)
        estimated_trajectory = np.array(self.estimated_trajectory)

        plt.figure(figsize=(10, 6))
        plt.plot(true_trajectory[:, 0], true_trajectory[:, 1], label='True trajectory', marker='.')
        plt.plot(estimated_trajectory[:, 0], estimated_trajectory[:, 1], label='Estimated trajectory', marker='x')
        plt.legend()
        plt.title('Extended Kalman Filter for 2D Localization with Lidar and Odometry')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    ekf_ros2_node = EKFRos2Node()

    try:
        rclpy.spin(ekf_ros2_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save both true and estimated trajectories
        ekf_ros2_node.save_trajectory_data()
        ekf_ros2_node.plot_trajectory()
        ekf_ros2_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
