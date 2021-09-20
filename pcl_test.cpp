#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

class Pcl_Filter : public rclcpp::Node
{
  public:
    Pcl_Filter()
    : Node("pcl_filter")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed2/zed_node/point_cloud/cloud_registered", 10, std::bind(&Pcl_Filter::topic_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("temp", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      // this converts the pointcloud2 message into something that pcl can use
      pcl::PCLPointCloud2 pc;
      sensor_msgs::msg::PointCloud2 pub_msg;
      pcl_conversions::toPCL(*msg, pc);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(pc,*temp_cloud);
      
      // do rotation on converted pointcloud message
      float rot = 1.57;
      // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
      // transform_2.rotate(Eigen::AngleAxisf(rot, Eigen::Vector3f(1,0,0)));
      // pcl::transformPointCloud (*temp_cloud, *rotated_cloud, transform_2);  

      // transform_2 = Eigen::Affine3f::Identity();
      // transform_2.rotate(Eigen::AngleAxisf(rot, Eigen::Vector3f(0,0,1)));
      // pcl::transformPointCloud (*rotated_cloud, *temp_cloud, transform_2);
      

      // filter z-axis for point cloud maximum
      pcl::PassThrough<pcl::PointXYZ> passmax;
      passmax.setInputCloud(temp_cloud);
      passmax.setFilterFieldName ("z");
      passmax.setFilterLimits (1.0, 10000);
      passmax.setFilterLimitsNegative (true);
      passmax.filter (*rotated_cloud);
      // filter z-axis for point cloud minimum
      pcl::PassThrough<pcl::PointXYZ> passmin;
      passmin.setInputCloud(rotated_cloud);
      passmin.setFilterFieldName ("z");
      passmin.setFilterLimits (-10000, -0.5);
      passmin.setFilterLimitsNegative (true);
      passmin.filter (*temp_cloud);
      // convert back to ROS msg to publish

      pcl::toROSMsg(*temp_cloud, pub_msg);
      pub_msg.header.frame_id = "map";
      publisher_->publish(pub_msg);
      RCLCPP_INFO(this->get_logger(), "filtered cloud");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pcl_Filter>());
  rclcpp::shutdown();
  return 0;
}
