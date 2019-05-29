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
}

void PackmlStacklight::callBackStatus(const packml_msgs::StatusConstPtr& msg)
{
  current_state_ = msg->state;
}

void PackmlStacklight::processCurState()
{
  StatusAction* status_ptr = getActionFromState(current_state_);

  std::map<std::string, uint8_t> temp_map = getPubMap(status_ptr);
  std::map<std::string, uint8_t>::iterator map_itr;
  for (map_itr = temp_map.begin(); map_itr != temp_map.end(); ++map_itr)
  {
    std_msgs::UInt8 msg;
    msg.data = map_itr->second;
    pub_map_[map_itr->first].publish<std_msgs::UInt8>(msg);
  }

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
