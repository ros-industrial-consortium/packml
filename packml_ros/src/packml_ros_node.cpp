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

#include <ros/ros.h>
#include <packml_ros/packml_ros.h>
#include <packml_sm/boost/packml_state_machine_continuous.h>

int myExecuteMethod()
{
  ROS_INFO_STREAM("This is my execute method(begin)");
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("This is my execute method(end)");
  return 0;  // returning zero indicates non-failure
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "packml_node");
  auto sm = packml_sm::PackmlStateMachineContinuous::spawn();
  sm->setExecute(std::bind(myExecuteMethod));

  packml_ros::PackmlRos sm_node(ros::NodeHandle(), ros::NodeHandle("~"), sm);
  sm_node.spin();

  return 0;
}
