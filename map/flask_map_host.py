import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from flask import Flask, jsonify, send_file
from threading import Thread
import numpy as np
import matplotlib.pyplot as plt
from io import BytesIO
# from matplotlib.colors import ListedColormap


app = Flask(__name__)

# Global variable to store the latest map data
latest_map_data = None
# cmap_values = ['green', 'gray', 'black']
# custom_cmap = ListedColormap(cmap_values)

# ROS 2 node to subscribe to the map topic and update the map data
class MapSubscriberNode(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',  # Replace with the actual topic name
            self.map_callback,
            10)

    def map_callback(self, msg):
        global latest_map_data
        # Convert OccupancyGrid message to a dictionary
        map_data = np.array(msg.data, dtype=np.uint8).reshape((msg.info.height, msg.info.width))
        latest_map_data = map_data

def publish_topic_data():
    global master_task_id_pub_glob
    rclpy.init()
    node = MapSubscriberNode()
    master_task_id_pub_glob = node
    rclpy.spin(node)

# API endpoint to get the latest map data
@app.route('/map', methods=['GET'])
def get_map_image():
    global latest_map_data
    if latest_map_data is not None:
        # Create a map image using Matplotlib
        plt.imshow(latest_map_data, cmap='gray')
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
    else:
        return jsonify({'error': 'Map data not available'})


def main():
    thread = Thread(target=publish_topic_data)
    thread.start()
    app.run(debug=True, threaded=True)

if __name__ == '__main__':
    main()
    
