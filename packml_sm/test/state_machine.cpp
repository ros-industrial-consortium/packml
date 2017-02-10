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
#include "packml_sm/common.h"
#include "packml_sm/events.h"
#include "packml_sm/state_machine.h"
#include "ros/duration.h"
#include "ros/console.h"
#include "ros/rate.h"

namespace packml_sm_test
{

using namespace packml_sm;

/**
 * @brief waitForState - returns true if the current state of the state machine (sm) matches the queried state
 * @param state - state enumeration to wait for
 * @param sm - top level state machine
 * @return true if state changes to queried state before internal timeout
 */
bool waitForState(StatesEnum state, StateMachine & sm)
{
  const double TIMEOUT = 2.0;
  const int SAMPLES = 50;
  ros::Rate r(double(SAMPLES)/TIMEOUT);
  for(int ii=0; ii<SAMPLES; ++ii)
  {
    if(sm.getCurrentState() == static_cast<int>(state))
    {
      ROS_INFO_STREAM("State changed to " << state);
      return true;
    }
    ROS_INFO_STREAM("Waiting for state to change to " << state);
    r.sleep();
  }
  return false;
}



TEST(Packml_SM, construction)
{
  ROS_INFO_STREAM("Construction test");
  StateMachine sm;
}

TEST(Packml_SM, state_diagram)
{
  ROS_INFO_STREAM("State diagram");
  StateMachine sm;
  sm.moveToThread(QCoreApplication::instance()->thread());
  EXPECT_FALSE(sm.isRunning());
  sm.start();

  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, sm));
  ASSERT_TRUE(sm.isRunning());

  sm.postEvent(CmdEvent::clear());
  ASSERT_TRUE(waitForState(StatesEnum::CLEARING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, sm));

  sm.postEvent(CmdEvent::reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, sm));

  sm.postEvent(CmdEvent::start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETE, sm));

  sm.postEvent(CmdEvent::reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, sm));

  sm.postEvent(CmdEvent::start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));

  sm.postEvent(CmdEvent::hold());
  ASSERT_TRUE(waitForState(StatesEnum::HOLDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::HELD, sm));

  sm.postEvent(CmdEvent::unhold());
  ASSERT_TRUE(waitForState(StatesEnum::UNHOLDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));

  sm.postEvent(CmdEvent::suspend());
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDED, sm));

  sm.postEvent(CmdEvent::unsuspend());
  ASSERT_TRUE(waitForState(StatesEnum::UNSUSPENDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));

  sm.postEvent(CmdEvent::stop());
  ASSERT_TRUE(waitForState(StatesEnum::STOPPING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, sm));

  sm.postEvent(CmdEvent::abort());
  ASSERT_TRUE(waitForState(StatesEnum::ABORTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, sm));

  sm.stop();
  ros::Duration(1).sleep();
  EXPECT_FALSE(sm.isRunning());
  ROS_INFO_STREAM("State diagram test complete");
}

}
