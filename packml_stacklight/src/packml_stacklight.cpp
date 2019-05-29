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

#include "packml_stacklight/packml_stacklight.h"
#include "packml_stacklight/utils.h"

namespace packml_stacklight
{
PackmlStacklight::PackmlStacklight(ros::NodeHandle nh, ros::NodeHandle pn) : nh_(nh), pn_(pn)
{
  ros::NodeHandle packml_node("~/packml");

  // get / set light on/off secs
  double val = 0.0;
  double default_val = 0.0;
  double (*get_val_fptr)() = nullptr;
  double (*set_val_fptr)(double) = nullptr;
  std::string param_name;

  param_name = "light_on_secs";
  get_val_fptr = getFlashingLightOnDur;
  set_val_fptr = setFlashingLightOnDur;
  default_val = get_val_fptr();
  val = 0.0;
  pn_.param(param_name, val, default_val);
  if (val != default_val)
  {
    set_val_fptr(val);
    ROS_INFO("changed %s => %.02f", param_name.c_str(), get_val_fptr());
  }
  else
  {
    ROS_INFO("default %s => %.02f", param_name.c_str(), get_val_fptr());
  }

  param_name = "light_off_secs";
  get_val_fptr = getFlashingLightOffDur;
  set_val_fptr = setFlashingLightOffDur;
  default_val = get_val_fptr();
  val = 0.0;
  pn_.param(param_name, val, default_val);
  if (val != default_val)
  {
    set_val_fptr(val);
    ROS_INFO("changed %s => %.02f", param_name.c_str(), get_val_fptr());
  }
  else
  {
    ROS_INFO("default %s => %.02f", param_name.c_str(), get_val_fptr());
  }

  // get / set buzzer on/off secs
  param_name = "buzzer_on_secs";
  get_val_fptr = getFlashingBuzzerOnDur;
  set_val_fptr = setFlashingBuzzerOnDur;
  default_val = get_val_fptr();
  val = 0.0;
  pn_.param(param_name, val, default_val);
  if (val != default_val)
  {
    set_val_fptr(val);
    ROS_INFO("changed %s => %.02f", param_name.c_str(), get_val_fptr());
  }
  else
  {
    ROS_INFO("default %s => %.02f", param_name.c_str(), get_val_fptr());
  }

  param_name = "buzzer_off_secs";
  get_val_fptr = getFlashingBuzzerOffDur;
  set_val_fptr = setFlashingBuzzerOffDur;
  default_val = get_val_fptr();
  val = 0.0;
  pn_.param(param_name, val, default_val);
  if (val != default_val)
  {
    set_val_fptr(val);
    ROS_INFO("changed %s => %.02f", param_name.c_str(), get_val_fptr());
  }
  else
  {
    ROS_INFO("default %s => %.02f", param_name.c_str(), get_val_fptr());
  }

  // get / set buttons multi_color default => false not multi color buttons //todo future
  param_name = "publish_frequency";
  get_val_fptr = getPublishFrequency;
  set_val_fptr = setPublishFrequency;
  default_val = get_val_fptr();
  val = 0.0;
  pn_.param(param_name, val, default_val);
  if (val != default_val)
  {
    set_val_fptr(val);
    ROS_INFO("changed %s => %.02f", param_name.c_str(), get_val_fptr());
  }
  else
  {
    ROS_INFO("default %s => %.02f", param_name.c_str(), get_val_fptr());
  }

  // get / set SUSPEND starving vs blocked
  param_name = "treat_suspend_starving";
  bool suspend_default = getSuspendStarving();
  bool suspend_val = suspend_default;
  pn_.param(param_name, suspend_val, suspend_default);
  if (suspend_val != suspend_default)
  {
    setSuspendStarving(suspend_val);
    ROS_INFO("changed %s => %.02f", param_name.c_str(), getSuspendStarving());
  }
  else
  {
    ROS_INFO("default %s => %.02f", param_name.c_str(), getSuspendStarving());
  }

  status_sub_ = packml_node.subscribe<packml_msgs::Status>("status", 1, &PackmlStacklight::callBackStatus, this,
                                                           ros::TransportHints().reliable().tcpNoDelay(true));

  packml_msgs::State temp;
  temp.val = packml_msgs::State::UNDEFINED;
  StatusAction* status_ptr = getActionFromState(temp);
  std::map<std::string, uint8_t> temp_map = getPubMap(status_ptr);
  std::map<std::string, uint8_t>::iterator map_itr;
  for (map_itr = temp_map.begin(); map_itr != temp_map.end(); ++map_itr)
  {
    pub_map_.insert(
        std::pair<std::string, ros::Publisher>(map_itr->first, pn.advertise<std_msgs::UInt8>(map_itr->first, 2, true)));
  }
}

PackmlStacklight::~PackmlStacklight()
{
  if (pub_map_.size() > 0)
  {
    pub_map_.clear();
  }
}

void PackmlStacklight::callBackStatus(const packml_msgs::StatusConstPtr& msg)
{
  current_state_ = msg->state;
}

void PackmlStacklight::processCurState()
{
  StatusAction* status_ptr = getActionFromState(current_state_);
  static std::map<std::string, uint8_t> last_map;

  std::map<std::string, uint8_t> temp_map = getPubMap(status_ptr);
  std::map<std::string, uint8_t>::iterator map_itr;
  for (map_itr = temp_map.begin(); map_itr != temp_map.end(); ++map_itr)
  {
    bool do_publish = false;
    if (doPublishAll(current_state_) || last_map.size() == 0)
    {
      do_publish = true;
    }
    else if (temp_map[map_itr->first] != last_map[map_itr->first])
    {
      do_publish = true;
    }

    if (do_publish)
    {
      std_msgs::UInt8 msg;
      msg.data = map_itr->second;
      pub_map_[map_itr->first].publish<std_msgs::UInt8>(msg);
    }
  }

  last_map = temp_map;

  return;
}

void PackmlStacklight::spin()
{
  while (ros::ok())
  {
    spinOnce();
    ros::Duration(0.001).sleep();
  }
}

void PackmlStacklight::spinOnce()
{
  ros::spinOnce();
  processCurState();
}

}  // namespace packml_stacklight
