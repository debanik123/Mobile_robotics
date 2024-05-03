import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from flask import Flask, jsonify, send_file, render_template, request
from threading import Thread
import numpy as np
import matplotlib.pyplot as plt
from io import BytesIO
from matplotlib.colors import ListedColormap
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math
import cv2

app = Flask(__name__)

# Global variable to store the latest map data
# latest_map_data = None
# latest_transform = None
p_x = p_y = latest_map_data = latest_transform = path = ros2_node = map_h = map_w = map_msg= None

cmap_values = ['white', 'black', 'white']
custom_cmap = ListedColormap(cmap_values)

# ROS 2 node to subscribe to the map topic and update the map data
class MapSubscriberNode(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',  # Replace with the actual topic name
            self.map_callback,
            10)
        
        self.subscription_path = self.create_subscription(
            Path,
            '/plan',  # Replace with the actual topic name
            self.path_callback,
            10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_to_pose_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.publisher_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(0.1, self.update_latest_transform)
        self.map_data = None

    def publish_goal_pose(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'  # Assuming the goal pose is in the 'map' frame
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        # Assuming theta is the yaw angle in radians
        goal_msg.pose.orientation.z = 1.2
        goal_msg.pose.orientation.w = 1.0
        # Publish the goal pose
        self.publisher_goal_pose.publish(goal_msg)
    
    
    def smooth_map_border(self, map_data):
        map_data_uint8 = np.uint8(map_data)
        smoothed_map = cv2.GaussianBlur(map_data_uint8, (5, 5), 0)
        return smoothed_map
    
    def map_callback(self, msg):
        global latest_map_data, map_h, map_w, map_msg
        # Convert OccupancyGrid message to a dictionary
        # print(msg.info.resolution, msg.info.width, msg.info.height, msg.info.origin.position.x, msg.info.origin.position.y)
        w = int(msg.info.height*msg.info.resolution)
        h = int(msg.info.width*msg.info.resolution)
        # print(msg.info.height, )
        map_data = np.array(msg.data, dtype=np.uint8).reshape((msg.info.height, msg.info.width))
        # map_data = self.smooth_map_border(map_data)
        latest_map_data = map_data
        self.map_data = msg
        map_w = self.map_data.info.width
        map_h = self.map_data.info.height
        map_msg = msg

    
    def update_latest_transform(self):
        transform = self.get_latest_transform()

        if transform is not None:
            global latest_transform
            latest_transform = transform
    
    def path_callback(self, msg):
        global path
        path = msg


    def get_latest_transform(self):
        global p_x, p_y
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time().to_msg())
            # print(transform)
            if self.map_data is not None:
                p_x, p_y = self.map_to_image_coordinates(transform.transform.translation.x, transform.transform.translation.y)
                # print(p_x, p_y)

            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Failed to lookup transform.")
            return None
        
    def map_to_image_coordinates(self, robot_x, robot_y):
        # Extract map information
        map_resolution = self.map_data.info.resolution
        map_origin_x = self.map_data.info.origin.position.x
        map_origin_y = self.map_data.info.origin.position.y
        image_width = self.map_data.info.width
        image_height = self.map_data.info.height

        # print(image_width, image_height, map_resolution)

        # Convert robot's map coordinates to image coordinates
        pixel_x = int((robot_x - map_origin_x) / map_resolution)
        pixel_y = int(image_height - (robot_y - map_origin_y) / map_resolution)  # Invert y-axis

        return pixel_x, pixel_y
    
    def image_to_map_coordinates(self, pixel_x, pixel_y):
        # Extract map information
        map_resolution = self.map_data.info.resolution
        map_origin_x = self.map_data.info.origin.position.x
        map_origin_y = self.map_data.info.origin.position.y
        image_width = self.map_data.info.width
        image_height = self.map_data.info.height

        # Convert image coordinates to robot's map coordinates
        robot_x = pixel_x * map_resolution + map_origin_x
        robot_y = ((image_height - pixel_y) * map_resolution) + map_origin_y  # Invert y-axis

        return robot_x, robot_y



def publish_topic_data():
    global ros2_node
    rclpy.init()
    node = MapSubscriberNode()
    ros2_node = node
    rclpy.spin(node)


# API endpoint to get the latest map data
@app.route('/map', methods=['GET'])
def get_map_image():
    global latest_map_data, p_x, p_y, ros2_node, path
    try:
        if latest_map_data is not None:
            if p_x is not None and p_y is not None:
                # print(p_x, p_y)
                plt.plot(p_x, p_y, 'ro', markersize=15)
            if ros2_node is not None:
                if path is not None:
                    for pose in path.poses:
                        # print(pose.pose.position.x, pose.pose.position.y)
                        pose_x, pose_y = ros2_node.map_to_image_coordinates(pose.pose.position.x, pose.pose.position.y)
                        # print(pose_x, pose_y)
                        plt.plot(pose_x, pose_y, 'g.', markersize=3)  # Plot path points

            plt.imshow(latest_map_data, cmap=custom_cmap, interpolation='nearest')
            plt.axis('off')  # Hide axes
            # Save the image to BytesIO buffer
            buffer = BytesIO()
            plt.savefig(buffer, format='png')
            buffer.seek(0)
            # Clear the plot to release memory
            plt.clf()
            plt.close()
            # Serve the image file from the buffer
            return send_file(buffer, mimetype='image/png')
            # return render_template('html/map.html')
    except Exception as e:
            # Handle any exceptions that may occur during the service call
        print(str(e))
        return jsonify({'error': 'Map data not available'})
    else:
        return jsonify({'error': 'Map data not available'})

@app.route('/map_viewer', methods=['GET'])
def map_viewer():
    return render_template('map_viewer.html')


@app.route('/click', methods=['POST'])
def handle_click():
    global map_h, map_w, map_msg
    data = request.json
    clicked_x = data['x']
    clicked_y = data['y']
    print("Clicked coordinates:", clicked_x, clicked_y)
    
    original_width = 640
    original_height = 480

    resized_width = map_w
    resized_height = map_h

    # Calculate the scaling factors
    scale_x = resized_width / original_width
    scale_y = resized_height / original_height

    # Resize the clicked coordinates
    resized_x = clicked_x * scale_x
    resized_y = clicked_y * scale_y

    print("Clicked coordinates (resized):", resized_x, resized_y)

    robot_x, robot_y = ros2_node.image_to_map_coordinates(resized_x, resized_y)
    print("robot_x --> ", robot_x, " robot_y --> ", robot_y)
    ros2_node.publish_goal_pose(robot_x, robot_y)
    # Process the clicked coordinates as needed
    return jsonify({'status': 'success'})



def main():
    thread = Thread(target=publish_topic_data)
    thread.start()
    app.run(host='0.0.0.0', port=9000,debug=True, threaded=False)

if __name__ == '__main__':
    main()
    
