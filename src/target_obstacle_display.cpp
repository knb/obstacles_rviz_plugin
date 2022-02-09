/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "target_obstacle_display.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

namespace obstacles_rviz_plugin
{

TargetObstacleDisplay::TargetObstacleDisplay() {
  circle_color_property_ = new rviz_common::properties::ColorProperty("Circles color", QColor(100, 20, 0), "Color of circles.", this, SLOT(updateCircleColor()));
  margin_color_property_ = new rviz_common::properties::ColorProperty("Margin color", QColor(0, 100, 20), "Color of margin added around circles.", this, SLOT(updateCircleColor()));
  alpha_property_ = new rviz_common::properties::FloatProperty("Opacity", 0.75, "Value 0,0 is fully transparent, 1,0 is fully opaque.", this, SLOT(updateAlpha()));
}

TargetObstacleDisplay::~TargetObstacleDisplay() {}

void TargetObstacleDisplay::onInitialize() {
  MFDClass::onInitialize();
}

void TargetObstacleDisplay::reset() {
  MFDClass::reset();
  circle_visuals_.clear();
}

void TargetObstacleDisplay::updateHistoryLength()
{
}

void TargetObstacleDisplay::updateCircleColor() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue main_color = circle_color_property_->getOgreColor();
  Ogre::ColourValue margin_color = margin_color_property_->getOgreColor();

  for (auto& c : circle_visuals_) {
    c->setMainColor(main_color.r, main_color.g, main_color.b, alpha);
    c->setMarginColor(margin_color.r, margin_color.g, margin_color.b, alpha);
  }
}

void TargetObstacleDisplay::updateAlpha() {
  updateCircleColor();
}

void TargetObstacleDisplay::processMessage(const cart_interfaces::msg::TargetObstacle::ConstSharedPtr target_obstacle_msg) {
  circle_visuals_.clear();

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(target_obstacle_msg->header.frame_id, target_obstacle_msg->header.stamp, position, orientation)) {
    RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Error transforming from frame '%s' to frame '%s'", target_obstacle_msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  const auto& circle = target_obstacle_msg->circle;
  std::shared_ptr<CircleVisual> visual;
  visual.reset(new CircleVisual(context_->getSceneManager(), scene_node_));

  visual->setData(circle);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  circle_visuals_.push_back(visual);

  updateAlpha();
}

} // end namespace obstacles_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(obstacles_rviz_plugin::TargetObstacleDisplay, rviz_common::Display)
