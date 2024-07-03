#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.hpp>

class NDTMapping : public rclcpp::Node {
public:
    NDTMapping()
    : Node("ndt_mapping_node"), ndt_(), initial_alignment_(false) {
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", 10, std::bind(&NDTMapping::pointCloudCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&NDTMapping::odomCallback, this, std::placeholders::_1));
        
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ndt_map", 10);

        // Set NDT parameters
        ndt_.setTransformationEpsilon(0.01);
        ndt_.setStepSize(0.1);
        ndt_.setResolution(1.0);
        ndt_.setMaximumIterations(35);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        
        // Downsample the cloud using a voxel grid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.2f, 0.2f, 0.2f);
        voxel_grid.filter(*filtered_cloud);

        if (!initial_alignment_) {
            // Initial alignment
            map_ = filtered_cloud;
            initial_alignment_ = true;
        } else {
            // Use NDT to align the new scan to the map
            ndt_.setInputSource(filtered_cloud);
            ndt_.setInputTarget(map_);

            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            ndt_.align(*aligned_cloud, guess_);

            if (ndt_.hasConverged()) {
                guess_ = ndt_.getFinalTransformation();
                *map_ += *aligned_cloud;
            }
        }

        // Publish the updated map
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*map_, output);
        output.header.frame_id = "map";
        map_pub_->publish(output);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Update the initial guess for the NDT algorithm based on odometry
        guess_ = Eigen::Matrix4f::Identity();
        guess_(0, 3) = msg->pose.pose.position.x;
        guess_(1, 3) = msg->pose.pose.position.y;
        guess_(2, 3) = msg->pose.pose.position.z;
        // Optionally, incorporate orientation if available
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_;
    Eigen::Matrix4f guess_;
    bool initial_alignment_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NDTMapping>());
    rclcpp::shutdown();
    return 0;
}
