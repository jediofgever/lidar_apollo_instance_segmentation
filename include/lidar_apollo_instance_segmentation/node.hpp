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

#ifndef LIDAR_APOLLO_INSTANCE_SEGMENTATION__NODE_HPP_
#define LIDAR_APOLLO_INSTANCE_SEGMENTATION__NODE_HPP_

#include "lidar_apollo_instance_segmentation/debugger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

#include <memory>

namespace lidar_apollo_instance_segmentation
{
    class LidarInstanceSegmentationInterface
    {
    public:
        LidarInstanceSegmentationInterface() {}
        virtual ~LidarInstanceSegmentationInterface() {}
        virtual bool detectDynamicObjects(
            const sensor_msgs::msg::PointCloud2 &input,
            vox_nav_msgs::msg::ObjectArray &output) = 0;
    };

    class LidarInstanceSegmentationNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Publisher<vox_nav_msgs::msg::ObjectArray>::SharedPtr dynamic_objects_pub_;
        std::shared_ptr<LidarInstanceSegmentationInterface> detector_ptr_;
        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr vision_detection_pub_;
        std::shared_ptr<Debugger> debugger_ptr_;
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

        void publisVisionMsg4DynamicObj(
            const vox_nav_msgs::msg::ObjectArray &input);

    public:
        explicit LidarInstanceSegmentationNode(const rclcpp::NodeOptions &node_options);
    };
} // namespace lidar_apollo_instance_segmentation

#endif // LIDAR_APOLLO_INSTANCE_SEGMENTATION__NODE_HPP_
