#ifndef CUT_POINTCLOUD_NODE_HPP_
#define CUT_POINTCLOUD_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "pointcloud_to_laserscan/visibility_control.h"

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pointcloud_to_laserscan
{
typedef tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> MessageFilter;

/**
* Class to process incoming pointclouds into laserscans.
* Some initial code was pulled from the defunct turtlebot pointcloud_to_laserscan implementation.
*/
class CutPointCloudNode : public rclcpp::Node
{
public:
  POINTCLOUD_TO_LASERSCAN_PUBLIC
  explicit CutPointCloudNode(const rclcpp::NodeOptions & options);

  ~CutPointCloudNode() override;

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);

  void subscriptionListenerThreadLoop();

  std::unique_ptr<tf2_ros::Buffer> tf2_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_;
  std::unique_ptr<MessageFilter> message_filter_;

  std::thread subscription_listener_thread_;
  std::atomic_bool alive_{true};

  // ROS Parameters
  int input_queue_size_;
  std::string target_frame_;
  double tolerance_;
  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_,
    range_max_;
  bool use_inf_;
  double inf_epsilon_;
  bool is_optical_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_pub_;
};

}  // namespace pointcloud_to_laserscan

#endif  // CUT_POINTCLOUD_NODE_HPP_
