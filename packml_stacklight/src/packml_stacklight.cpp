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
PackmlStacklight::PackmlStacklight(ros::NodeHandle nh, ros::NodeHandle pn) : nh_(nh), pn_(pn)
{
  ros::NodeHandle packml_node("~/packml");

  status_sub_ = packml_node.subscribe<packml_msgs::Status>("status", 1, &PackmlStacklight::callBackStatus, this,
                                                           ros::TransportHints().reliable().tcpNoDelay(true));
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
  static ros::Time last_time = ros::Time::now();

  // about every X seconds
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
