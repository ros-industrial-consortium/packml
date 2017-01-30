/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016 Shaun Edwards
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

#include <gtest/gtest.h>
#include "packml_sm/state_machine.h"
#include "ros/duration.h"
#include "ros/console.h"

namespace packml_sm_test
{

using namespace packml_sm;

TEST(Packml_SM, construction)
{
  ROS_INFO_STREAM("Construction test");
  StateMachine sm;
  sm.moveToThread(QCoreApplication::instance()->thread());
  EXPECT_FALSE(sm.isRunning());
  sm.start();
  ROS_INFO("Gave command to start sm");
  ros::Duration(1).sleep();
  EXPECT_TRUE(sm.isRunning());
  sm.postEvent(CmdEvent::clear());
  QCoreApplication::instance()->processEvents();
  ROS_INFO("Called process events");
  sm.stop();
  ros::Duration(1).sleep();
  EXPECT_FALSE(sm.isRunning());
  ROS_INFO_STREAM("Construction test complete");
}

}
