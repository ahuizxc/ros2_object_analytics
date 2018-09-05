
/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <object_analytics_msgs/msg/tracked_objects.hpp>
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include "moving_object/moving_object_ros.hpp"
#include "object_analytics_node/const.hpp"

namespace object_analytics_node
{
namespace moving_object
{
MovingObjectRos::MovingObjectRos() : Node("moving_object")
{
  RCLCPP_INFO(get_logger(), "Entering MovingObjectRos Constructor...");
  f_detection_sub_ = std::make_unique<FilteredDetection>(this, Const::kTopicDetection);
  f_tracking_sub_ = std::make_unique<FilteredTracking>(this, Const::kTopicTracking);
  f_localization_sub_ =
      std::make_unique<FilteredLocalization>(this, Const::kTopicLocalization);

  sync_sub_ =
      sub_sync_seg = std::unique_ptr<FilteredSync>(new FilteredSync(FilteredPolicy(kMsgQueueSize), *f_detection_sub_, *f_tracking_sub_, *f_localization_sub_, 10));
      //std::make_unique<FilteredSync>(*f_detection_sub_, *f_tracking_sub_, *f_localization_sub_, 10);

  sync_sub_->registerCallback(std::bind(&MovingObjectRos::onObjectsReceived, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(get_logger(), "...Creating Moving Objects buffer...");
  frames_ = std::make_shared<MovingObjects>(rclcpp::Node::SharedPtr(this), params_);
  RCLCPP_INFO(get_logger(), "...message_detction:%s, tracking:%s, localization:%s",
              kTopicDetection, kTopicDetection,
              kTopicDetection);
}

void MovingObjectRos::onObjectsReceived(const DetectionMsg::SharedPtr& detect,
                                        const TrackingMsg::SharedPtr& track,
                                        const LocalizationMsg::SharedPtr& loc)
{
  if (loc->header.stamp != track->header.stamp || track->header.stamp != detect->header.stamp ||
      loc->header.stamp != detect->header.stamp)
  {
    RCLCPP_WARN(get_logger(), "...Doesn't meet the stamp check, do nothing for the current \
    messages");
    RCLCPP_WARN(get_logger(), "......D==%ld.%ld, T==%ld.%ld, L==%ld.%ld", detect->header.stamp.sec,
                detect->header.stamp.nanosec, track->header.stamp.sec, track->header.stamp.nanosec,
                loc->header.stamp.sec, loc->header.stamp.nanosec);

    return;
  }
  if (loc->header.frame_id != track->header.frame_id ||
      track->header.frame_id != detect->header.frame_id ||
      loc->header.frame_id != detect->header.frame_id)
  {
    RCLCPP_WARN(get_logger(), "...Doesn't meet the frame_id check, do nothing for the current \
messages");

    return;
  }
  frames_->processFrame(detect, track, loc);
}

bool MovingObjectRos::setParameters(const std::shared_ptr<Param> params)
{
  if (params != nullptr)
  {
    params_ = params;
    return true;
  }
  return false;
}
}  // namespace moving object
}  // namespace object_analytics_node

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(object_analytics_node::segmenter::MovingObjectRos, rclcpp::Node)