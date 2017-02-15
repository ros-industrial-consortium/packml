/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017 Shaun Edwards
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

#include "packml_ros/packml_ros.h"
#include "packml_msgs/utils.h"

namespace packml_ros
{


PackmlRos::PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn,
          std::shared_ptr<packml_sm::StateMachine> sm) :
 nh_(nh), pn_(pn)
{
  ros::NodeHandle packml_node("packml");

  status_pub_ = packml_node.advertise<packml_msgs::Status>("status", 10);
  status_ = packml_msgs::initStatus();

  connect(sm.get(), SIGNAL(stateChanged(int, QString)), this, SLOT(pubState(int, QString)));
}

void PackmlRos::spin()
{
  while(ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE(0.5, "PackmlRos::spin()");
    spinOnce();
  }
  return;
}


void PackmlRos::spinOnce()
{
  QCoreApplication::instance()->processEvents();
  ros::spinOnce();
}


void PackmlRos::pubState(int value, QString name)
{
  ROS_INFO_STREAM("Publishing state change: " << name.toStdString() << "(" << value << ")");

  if( packml_msgs::isStandardState(value) )
  {
    status_.state.val = value;
  }
  else
  {
    status_.sub_state = value;
  }
  status_pub_.publish(status_);

}

} // namespace kitsune_robot
