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

namespace packml_stacklight
{
void PackmlStacklight::setDoubleParam(std::string param_name, double& default_val)
{
  double val = 0.0;
  std::string msg = "";
  pn_.param(param_name, val, default_val);
  if (val != default_val)
  {
    default_val = val;
    msg = "changed";
  }
  else
  {
    msg = "default";
  }

  ROS_INFO("%s %s %s => %.02f", __FUNCTION__, msg.c_str(), param_name.c_str(), default_val);
}

void PackmlStacklight::setBoolParam(std::string param_name, bool& default_val)
{
  bool val = false;
  std::string msg = "";
  pn_.param(param_name, val, default_val);
  if (val != default_val)
  {
    default_val = val;
    msg = "changed";
  }
  else
  {
    msg = "default";
  }

  ROS_INFO("%s %s %s => %s", __FUNCTION__, msg.c_str(), param_name.c_str(), default_val ? "true" : "false");
}

PackmlStacklight::PackmlStacklight(ros::NodeHandle nh, ros::NodeHandle pn) : nh_(nh), pn_(pn)
{
  setDoubleParam("light_on_secs", utils_.flash_sec_light_on_);
  setDoubleParam("light_off_secs", utils_.flash_sec_light_off_);
  setDoubleParam("buzzer_on_secs", utils_.flash_sec_buzzer_on_);
  setDoubleParam("buzzer_off_secs", utils_.flash_sec_buzzer_off_);
  setDoubleParam("publish_frequency", utils_.publish_frequency_);

  bool suspend_default = utils_.getSuspendStarving();
  setBoolParam("treat_suspend_starving", suspend_default);
  utils_.setSuspendStarving(suspend_default);

  status_sub_ = nh_.subscribe<packml_msgs::Status>("status", 1, &PackmlStacklight::callBackStatus, this);

  std::map<std::string, uint8_t> temp_map = utils_.getPubMap(current_state_);
  std::map<std::string, uint8_t>::iterator map_itr;
  for (map_itr = temp_map.begin(); map_itr != temp_map.end(); ++map_itr)
  {
    pub_map_.insert(std::pair<std::string, ros::Publisher>(map_itr->first,
                                                           nh_.advertise<std_msgs::UInt8>(map_itr->first, 2, true)));
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
  static std::map<std::string, uint8_t> last_map;
  std::map<std::string, uint8_t>::iterator map_itr;

  std::map<std::string, uint8_t> temp_map = utils_.getPubMap(current_state_);
  for (map_itr = temp_map.begin(); map_itr != temp_map.end(); ++map_itr)
  {
    bool do_publish = false;
    if (utils_.getShouldPublish(current_state_) || last_map.size() == 0)
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
