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
#include <ros/time.h>
#include <ros/console.h>
#include <packml_msgs/TransitionRequest.h>
#include <packml_msgs/State.h>
#include <packml_sm/common.h>


int main(int argc, char **argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


TEST(Packml_ROS, sync_packml_cmd)
{
  using namespace packml_msgs;
  using namespace packml_sm;
  EXPECT_EQ(static_cast<int>(CmdEnum::ABORT), TransitionRequest::Request::ABORT);
  EXPECT_EQ(static_cast<int>(CmdEnum::CLEAR), TransitionRequest::Request::CLEAR);
  EXPECT_EQ(static_cast<int>(CmdEnum::ESTOP), TransitionRequest::Request::ESTOP);
  EXPECT_EQ(static_cast<int>(CmdEnum::HOLD), TransitionRequest::Request::HOLD);
  EXPECT_EQ(static_cast<int>(CmdEnum::RESET), TransitionRequest::Request::RESET);
  EXPECT_EQ(static_cast<int>(CmdEnum::START), TransitionRequest::Request::START);
  EXPECT_EQ(static_cast<int>(CmdEnum::STOP), TransitionRequest::Request::STOP);
  EXPECT_EQ(static_cast<int>(CmdEnum::SUSPEND), TransitionRequest::Request::SUSPEND);
  EXPECT_EQ(static_cast<int>(CmdEnum::UNDEFINED), TransitionRequest::Request::UNDEFINED);
  EXPECT_EQ(static_cast<int>(CmdEnum::UNHOLD), TransitionRequest::Request::UNHOLD);
  EXPECT_EQ(static_cast<int>(CmdEnum::UNSUSPEND), TransitionRequest::Request::UNSUSPEND);
}


TEST(Packml_ROS, sync_packml_state)
{
  using namespace packml_msgs;
  using namespace packml_sm;
  EXPECT_EQ(static_cast<int>(StatesEnum::ABORTED), State::ABORTED);
  EXPECT_EQ(static_cast<int>(StatesEnum::ABORTING), State::ABORTING);
  EXPECT_EQ(static_cast<int>(StatesEnum::CLEARING), State::CLEARING);
  EXPECT_EQ(static_cast<int>(StatesEnum::COMPLETE), State::COMPLETE);
  EXPECT_EQ(static_cast<int>(StatesEnum::COMPLETING), State::COMPLETING);
  EXPECT_EQ(static_cast<int>(StatesEnum::EXECUTE), State::EXECUTE);
  EXPECT_EQ(static_cast<int>(StatesEnum::HELD), State::HELD);
  EXPECT_EQ(static_cast<int>(StatesEnum::HOLDING), State::HOLDING);
  EXPECT_EQ(static_cast<int>(StatesEnum::IDLE), State::IDLE);

}
