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

#ifndef OBSTACLES_RVIZ_PLUGIN_TARGET_OBSTACLE_DISPLAY_HPP
#define OBSTACLES_RVIZ_PLUGIN_TARGET_OBSTACLE_DISPLAY_HPP

// #ifndef Q_MOC_RUN
// #include <OgreSceneNode.h>
// #include <OgreSceneManager.h>

#include <cart_interfaces/msg/target_obstacle.hpp>
#include <tf2_ros/transform_listener.h>

// #include <rviz_common/properties/color_property.hpp>
// #include <rviz_common/properties/float_property.hpp>
// #include <rviz_common/visualization_manager.hpp>
#include <rviz_common/message_filter_display.hpp>
// #include <rviz_common/ros_topic_display.hpp>

#include "obstacles_rviz_plugin/circle_visual.h"
// #include "obstacles_rviz_plugin/segment_visual.h"
// #endif

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
}
}

namespace obstacles_rviz_plugin
{

class TargetObstacleDisplay : public rviz_common::MessageFilterDisplay<cart_interfaces::msg::TargetObstacle>
{
  Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  TargetObstacleDisplay();

  virtual ~TargetObstacleDisplay();

  virtual void onInitialize() override;
  virtual void reset() override;

private Q_SLOTS:
  void updateCircleColor();
  void updateAlpha();
  void updateHistoryLength();

private:
  virtual void processMessage(const cart_interfaces::msg::TargetObstacle::ConstSharedPtr target_obstacle_msg) override;

  std::vector< std::shared_ptr<CircleVisual> > circle_visuals_;

  rviz_common::properties::ColorProperty* circle_color_property_;
  rviz_common::properties::ColorProperty* margin_color_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
};

} // end namespace obstacles_display

#endif
