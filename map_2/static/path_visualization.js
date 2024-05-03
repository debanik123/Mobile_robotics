var maps = {}; // Dictionary to store maps and their canvas elements
var canvas, ctx, mapData, scaleX, scaleY;
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


mapview.subscribe(function(map_msg) {
  const mapName = mapview.name; // Assuming topic name represents map name
  console.log(`Received map data for: ${mapName}`);
  mapData = map_msg;

  if (!maps[mapName]) {
    canvas = createCanvas(mapName);
    maps[mapName].ctx = canvas.getContext('2d');
  }

  canvas = maps[mapName].canvas;
  ctx = maps[mapName].ctx;

  // Update canvas dimensions based on map data
  canvas.width = 480;
  canvas.height = 480;

  // Clear previous map visualization (optional)
  clearCanvas(mapName);

  scaleX = 480 / map_msg.info.width;
  scaleY = 480 / map_msg.info.height;

  for (var y = 0; y < map_msg.info.height; y++) {
    for (var x = 0; x < map_msg.info.width; x++) {
      var index = x + y * map_msg.info.width;
      var value = map_msg.data[index];
      if (value === 100) {
        // Occupied space
        ctx.fillStyle = 'black';
      } else if (value === 0) {
        // Free space
        ctx.fillStyle = 'white';
      } else {
        // Unknown space
        ctx.fillStyle = 'gray';
      }
      // ctx.fillRect(x, y, 1, 1);
      ctx.fillRect(x * scaleX, y * scaleY, scaleX, scaleY);
    }
  }
});

pathSubscriber.subscribe(function(pathMsg) {
    // ctx.clearRect(0, 0, canvas.width, canvas.height);  // Clear previous path
    visualizePath(pathMsg.poses);
    // console.log(pathMsg.poses);
  });

function visualizePath(poses) {
    for (let i = 0; i < poses.length - 1; i++) {
        const pose1 = poses[i].pose.position;
        const pose2 = poses[i + 1].pose.position;
        
        // console.log(`Path Segment ${i+1}:`);
        // console.log(`Pose 1: (${pose1.x}, ${pose1.y})`);
        // console.log(`Pose 2: (${pose2.x}, ${pose2.y})`);

        const imageCoords1 = mapToImageCoordinates(pose1.x, pose1.y);
        const imageCoords2 = mapToImageCoordinates(pose2.x, pose2.y);

        ctx.beginPath();
        ctx.moveTo(imageCoords1.x, imageCoords1.y);
        ctx.lineTo(imageCoords2.x, imageCoords2.y);
        ctx.stroke();

        // console.log("Image coordinates1:", imageCoords1);
        // console.log("Image coordinates2:", imageCoords2);
        // console.log("--------------------");
    }
}


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

    return { x: pixel_x*scaleX, y: pixel_y*scaleY};
}

// ROS connection events
ros.on('connected', function() {
  console.log('Connected to ROS server');
});

ros.on('error', function(error) {
  console.error('Error connecting to ROS server:', error);
});
