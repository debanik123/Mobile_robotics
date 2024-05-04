var maps = {}; // Dictionary to store maps and their canvas elements
var canvas, ctx, mapData, scaleX, scaleY;
var mapName;
var p1_x = null;
var p1_y = null;
var path_g = null;

// ros2 run rosbridge_server rosbridge_websocket
// ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
// ROS connection setup (assuming ROSLIB is already included)
var ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'  // Replace with your ROS bridge server address
});

// Map visualization functions
function createCanvas(mapName) {
  const mapContainer = document.getElementById('map-container');
  canvas = document.createElement('canvas');
  canvas.id = `map-canvas-${mapName}`;
  canvas.width = 0; // Will be set later
  canvas.height = 0; // Will be set later
  mapContainer.appendChild(canvas);
  maps[mapName] = { canvas: canvas };
  return canvas;
}

function clearCanvas(mapName) {
  if (maps[mapName]) {
    canvas = maps[mapName].canvas;
    ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
  }
}

// ROS map subscription (adjust topic name and message type if needed)
var mapview = new ROSLIB.Topic({
  ros: ros,
  name: '/map', // Subscribe to all map topics (replace with specific topic if needed)
  messageType: 'nav_msgs/OccupancyGrid'  // Adjust based on your map message type
});

var pathSubscriber = new ROSLIB.Topic({
    ros : ros,
    name : '/plan',
    messageType : 'nav_msgs/Path'
});

// Create the tf2Subscriber
var tf2Subscriber = new ROSLIB.Topic({
  ros: ros,
  name: '/tf',
  messageType: 'tf2_msgs/msg/TFMessage'
});

tf2Subscriber.subscribe(function(msg) {
  // Message handling logic here
  // console.log('Received TF2 message:', msg.transforms);

  for (const transform of msg.transforms) {
    
    console.log('Received TF2 message:', transform.header.frame_id);
    const translation = transform.transform.translation;
    const rotation = transform.transform.rotation;
    
    console.log('Received transform:');
    console.log('Translation:', translation);
    console.log('Rotation:', rotation);
    

  }
  //   // Check if the transform is between map and base_footprint
  //   if (transform.header.frame_id === 'map' && transform.child_frame_id === 'base_footprint') {
  //       // Extract translation and rotation
        // const translation = transform.transform.translation;
        // const rotation = transform.transform.rotation;

  //       // Log the transform data
        // console.log('Received transform:');
        // console.log('Translation:', translation);
        // console.log('Rotation:', rotation);
  //   }
  // }
});

mapview.subscribe(function(map_msg) {
  mapName = mapview.name; // Assuming topic name represents map name
  console.log(`Received map data for: ${mapName}`);
  mapData = map_msg;

  if (!maps[mapName]) {
    canvas = createCanvas(mapName);
    maps[mapName].ctx = canvas.getContext('2d');
    console.log(`Inside: ${mapName}`);

    canvas = maps[mapName].canvas;
    ctx = maps[mapName].ctx;

    // Update canvas dimensions based on map data
    canvas.width = 480;
    canvas.height = 480;

    // Clear previous map visualization (optional)
    // clearCanvas(mapName);

    scaleX = canvas.width / map_msg.info.width;
    scaleY = canvas.height / map_msg.info.height;
    
    visualizeMap(map_msg);
  }

  
});

function visualizeMap(map_msg) {
  for (var y = 0; y < map_msg.info.height; y++) {
      for (var x = 0; x < map_msg.info.width; x++) {
          var index = x + y * map_msg.info.width;
          var value = map_msg.data[index];
          var color = getColorForOccupancy(value);
          ctx.fillStyle = color;
          // ctx.fillRect(x, y, 1, 1);
          ctx.fillRect(x * scaleX, y * scaleY, scaleX, scaleY);
      }
  }

  if (path_g !== null) 
  {
    // drawFilledCircle(ctx, p1_x, p1_y, 5, 'red');
    visualizePath(path_g);
  }
}

function getColorForOccupancy(occupancyValue) {
  if (occupancyValue === 100) {
      return 'black'; // Occupied space
  } else if (occupancyValue === 0) {
      return 'white'; // Free space
  } else {
      // Calculate grayscale color based on occupancy value
      var colorValue = 255 - (occupancyValue * 255) / 100;
      return 'rgb(' + colorValue + ',' + colorValue + ',' + colorValue + ')';
  }
}

pathSubscriber.subscribe(function(pathMsg) { 
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  path_g = pathMsg.poses
  
  visualizeMap(mapData);
    // console.log(pathMsg.poses);
});

function visualizePath(poses) {
    ctx.strokeStyle = 'green';
    // const darkGreen = '#006400'; // You can adjust the hex code as needed
    // ctx.strokeStyle = darkGreen;
    ctx.lineWidth = 5;

    for (let i = 0; i < poses.length - 1; i++) {
        const pose1 = poses[i].pose.position;
        const pose2 = poses[i + 1].pose.position;
        
        // console.log(`Path Segment ${i+1}:`);
        // console.log(`Pose 1: (${pose1.x}, ${pose1.y})`);
        // console.log(`Pose 2: (${pose2.x}, ${pose2.y})`);

        const imageCoords1 = mapToImageCoordinates(pose1.x, pose1.y);
        const imageCoords2 = mapToImageCoordinates(pose2.x, pose2.y);

        // p1_x = parseInt(imageCoords1.x); 
        // p1_y = parseInt(imageCoords1.y);
        // console.log(`Pose 1: (${p1_x}, ${p1_y})`);

        ctx.beginPath();
        ctx.moveTo(imageCoords1.x, imageCoords1.y);
        ctx.lineTo(imageCoords2.x, imageCoords2.y);
        ctx.stroke();
        // drawFilledCircle(ctx, imageCoords1.x, imageCoords1.y, 1, 'red');
    }
    
}

// Example usage:
// Assuming you have a canvas context named ctx
// and you want to draw a filled circle at coordinates (100, 100) with a radius of 50 and red color
// tfClient.subscribe('/tf', handleTFMessage);

// function handleTFMessage(tfMsg) {
//   console.log('Received TF message:', tfMsg);

//   // Access specific transforms from the message (tfMsg.transforms)
//   // ... your logic to process and visualize the transforms
// }

function mapToImageCoordinates(robot_x, robot_y) {
    // Extract map information
    const map_resolution = mapData.info.resolution;
    const map_origin_x = mapData.info.origin.position.x;
    const map_origin_y = mapData.info.origin.position.y;
    const image_width = mapData.info.width;
    const image_height = mapData.info.height;

    // Convert robot's map coordinates to image coordinates
    const pixel_x = Math.floor((robot_x - map_origin_x) / map_resolution);
    const pixel_y = Math.floor(image_height - (robot_y - map_origin_y) / map_resolution);  // Invert y-axis

    // return { x: pixel_x, y: pixel_y };
    return { x: pixel_x * scaleX, y: pixel_y * scaleY };
}

// ROS connection events
ros.on('connected', function() {
  console.log('Connected to ROS server');
});

ros.on('error', function(error) {
  console.error('Error connecting to ROS server:', error);
});
