/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2019 Joshua Hatzenbuehler
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PACKML_STACKLIGHT_H
#define PACKML_STACKLIGHT_H

#include <ros/ros.h>
#include <packml_msgs/Status.h>
#include <packml_msgs/State.h>
#include <std_msgs/UInt8.h>
#include "packml_stacklight/utils.h"

namespace packml_stacklight
{
class PackmlStacklight
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pn_;
  ros::Subscriber status_sub_;
  std::map<std::string, ros::Publisher> pub_map_;

private:
  packml_msgs::State current_state_;
  packml_stacklight::Utils utils_;
  ros::Time current_state_time_;

public:
  PackmlStacklight(ros::NodeHandle nh, ros::NodeHandle pn);
  ~PackmlStacklight();

  void spin();
  void spinOnce();

private:
  void callBackStatus(const packml_msgs::StatusConstPtr& msg);
  void processCurState();
  void setDoubleParam(std::string param_name, double& default_val);
  void setBoolParam(std::string param_name, bool& default_val);
};
}  // namespace packml_stacklight

#endif  // PACKML_STACKLIGHT_H