/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/cut_pointcloud_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

CutPointCloudNode::CutPointCloudNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cut_pointcloud", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  is_optical_ = this->declare_parameter("is_optical", true);

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cut_cloud", rclcpp::SystemDefaultsQoS());

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&CutPointCloudNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&CutPointCloudNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&CutPointCloudNode::subscriptionListenerThreadLoop, this));
}

CutPointCloudNode::~CutPointCloudNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void CutPointCloudNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = cloud_pub_->get_subscription_count() +
      cloud_pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to cutted pointcloud, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to  cutted pointcloud, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void CutPointCloudNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  auto start = std::chrono::high_resolution_clock::now();
  // build laserscan output

//https://github.com/ros2/turtlebot2_demo/blob/master/depthimage_to_pointcloud2/src/depthimage_to_pointcloud2_node.cpp
//https://github.com/ros2/turtlebot2_demo/blob/master/depthimage_to_pointcloud2/include/depthimage_to_pointcloud2/depth_conversions.hpp

  // Transform cloud if necessary
  if (is_optical_){
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp = cloud_msg->header.stamp;
    cloud->header.frame_id = target_frame_;
    cloud->width = cloud_msg->width;
    cloud->height = cloud_msg->height;
    cloud->point_step = cloud_msg->point_step;
    cloud->row_step = cloud_msg->row_step;
    cloud->is_bigendian = cloud_msg->is_bigendian;
    cloud->is_dense = cloud_msg->is_dense;
    
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    //modifier.resize(cloud_msg->width);
    sensor_msgs::PointCloud2ConstIterator<uint8_t>  iter_r(*cloud_msg, "r"), iter_g(*cloud_msg, "g"), iter_b(*cloud_msg, "b");
    sensor_msgs::PointCloud2Iterator<uint8_t>  iter_cr(*cloud, "r"), iter_cg(*cloud, "g"), iter_cb(*cloud, "b");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"), iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_cx(*cloud, "x"), iter_cy(*cloud, "y"), iter_cz(*cloud, "z");
    for (;
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b, ++iter_cx, ++iter_cy, ++iter_cz, ++iter_cr, ++iter_cg, ++iter_cb)
    {
      *iter_cx = *iter_z;
      *iter_cy = -(*iter_x);
      *iter_cz = -(*iter_y);
      *iter_cr = *iter_r;
      *iter_cg = *iter_g;
      *iter_cb = *iter_b;
    }
    cloud_msg = cloud;
    
  }
  else if (target_frame_ != cloud_msg->header.frame_id  && !is_optical_) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }
  

  // initialize cutted pointcloud message
  
  auto cut_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  // Iterate through pointcloud
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"), iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t>  iter_r(*cloud_msg, "r"), iter_g(*cloud_msg, "g"), iter_b(*cloud_msg, "b");
 
  for (;
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }
    
    double range = pow(pow(double (*iter_x), 2.0) + pow(double (*iter_y), 2.0) + pow(double (*iter_z), 2.0), 0.5);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < angle_min_ || angle > angle_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, angle_min_, angle_max_);
      continue;
    }
    //uint8_t transparancy = 0;
    //auto rgba = std::make_shared<uint32_t>(((*iter_r)<<24) + ((*iter_g)<<16) + ((*iter_b)<<8) + transparancy);
    // uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    //*uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    //*p.rgb = *reinterpret_cast<float*>(&rgb);
    auto point = std::make_shared<pcl::PointXYZRGB>(*iter_r, *iter_g, *iter_b);
    point->x = float (*iter_x);
    point->y = float (*iter_y);
    point->z = float (*iter_z);
    (*cut_cloud).push_back(*point);

  }
  auto cut_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cut_cloud, *cut_cloud_msg);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
  cut_cloud_msg->header.frame_id = target_frame_;
  cut_cloud_msg->header.stamp = rclcpp::Time(cloud_msg->header.stamp) + rclcpp::Duration(duration);
  

  std::string out;
  out = std::to_string(duration.count()) + " ms";
  
  RCLCPP_INFO(this->get_logger(), out);
  cloud_pub_->publish(*cut_cloud_msg);
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::CutPointCloudNode)
