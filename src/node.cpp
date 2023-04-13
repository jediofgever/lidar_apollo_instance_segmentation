// Copyright 2020-2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lidar_apollo_instance_segmentation/node.hpp"
#include "lidar_apollo_instance_segmentation/detector.hpp"

namespace lidar_apollo_instance_segmentation
{
  LidarInstanceSegmentationNode::LidarInstanceSegmentationNode(
      const rclcpp::NodeOptions &node_options)
      : Node("lidar_apollo_instance_segmentation_node", node_options)
  {
    using std::placeholders::_1;

    detector_ptr_ = std::make_shared<LidarApolloInstanceSegmentation>(this);
    debugger_ptr_ = std::make_shared<Debugger>(this);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input/pointcloud", rclcpp::SensorDataQoS(),
        std::bind(&LidarInstanceSegmentationNode::pointCloudCallback, this, _1));

    dynamic_objects_pub_ =
        this->create_publisher<vox_nav_msgs::msg::ObjectArray>("output/labeled_clusters", rclcpp::QoS{1});

    vision_detection_pub_ =
        this->create_publisher<vision_msgs::msg::Detection3DArray>("output/labeled_detections", rclcpp::QoS{1});

    if (get_parameter("use_sim_time").as_bool())
    {
      RCLCPP_INFO(this->get_logger(), "Using simulation time as use_sim_time is set to true");
    }
  }

  void LidarInstanceSegmentationNode::pointCloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    vox_nav_msgs::msg::ObjectArray output_msg;
    detector_ptr_->detectDynamicObjects(*msg, output_msg);
    dynamic_objects_pub_->publish(output_msg);
    debugger_ptr_->publishColoredPointCloud(output_msg);
    publisVisionMsg4DynamicObj(output_msg);
  }

  void LidarInstanceSegmentationNode::publisVisionMsg4DynamicObj(
      const vox_nav_msgs::msg::ObjectArray &input)
  {
    // RVIZ visualization of dynamic objects
    vision_msgs::msg::Detection3DArray output_msg;
    output_msg.header = input.header;

    for (const auto &obj : input.objects)
    {
      vision_msgs::msg::Detection3D detection;
      detection.header = input.header;
      detection.bbox.center.position.x = obj.pose.position.x;
      detection.bbox.center.position.y = obj.pose.position.y;
      detection.bbox.center.position.z = obj.pose.position.z;
      detection.bbox.center.orientation.x = obj.pose.orientation.x;
      detection.bbox.center.orientation.y = obj.pose.orientation.y;
      detection.bbox.center.orientation.z = obj.pose.orientation.z;
      detection.bbox.center.orientation.w = obj.pose.orientation.w;
      detection.bbox.size.x = obj.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
      detection.bbox.size.y = obj.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
      detection.bbox.size.z = obj.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
      detection.id = obj.id;

      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = obj.classification_label;
      // make sure the score is not NAN or INF
      if (std::isnan(obj.classification_probability) || std::isinf(obj.classification_probability))
      {
        hypothesis.hypothesis.score = -1.0;
      }
      else
      {
        hypothesis.hypothesis.score = obj.classification_probability;
      }
      hypothesis.pose.pose = obj.pose;
      detection.results.push_back(hypothesis);

      // If any of the dimensions is NAN or INF, skip this detection
      if (std::isnan(detection.bbox.size.x) || std::isinf(detection.bbox.size.x) ||
          std::isnan(detection.bbox.size.y) || std::isinf(detection.bbox.size.y) ||
          std::isnan(detection.bbox.size.z) || std::isinf(detection.bbox.size.z))
      {
        continue;
      }

      output_msg.detections.push_back(detection);
    }
    vision_detection_pub_->publish(output_msg);
  }

} // namespace lidar_apollo_instance_segmentation

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_apollo_instance_segmentation::LidarInstanceSegmentationNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(lidar_apollo_instance_segmentation::LidarInstanceSegmentationNode)
