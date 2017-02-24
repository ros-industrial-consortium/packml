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
    ROS_DEBUG_STREAM("Waiting for state to change to " << state);
    r.sleep();
  }
  return false;
}

int success()
{
  ROS_INFO_STREAM("Beginning success() method");
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("Execute method complete");
  return 0;
}

int fail()
{
  ROS_INFO_STREAM("Beginning fail method");
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("Execute method complete");
  return -1;
}


TEST(Packml_SM, set_execute)
{
  std::shared_ptr<StateMachine> sm = StateMachine::singleCyleSM();
  sm->setExecute(std::bind(success));
  sm->activate();
  ros::Duration(1.0).sleep();  //give time to start
  ASSERT_TRUE(sm->clear());
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, *sm));
  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, *sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETE, *sm));
  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, *sm));
  sm->setExecute(std::bind(fail));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, *sm));
}

TEST(Packml_SC, state_diagram)
{
  ROS_INFO_STREAM("SINGLE CYCLE::State diagram");
  SingleCycle sm;
  EXPECT_FALSE(sm.isActive());
  sm.setExecute(std::bind(success));
  sm.activate();
  ros::Duration(1.0).sleep();  //give time to start
  EXPECT_TRUE(sm.isActive());

  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, sm));
  ASSERT_TRUE(sm.isActive());

  ASSERT_TRUE(sm.clear());
  ASSERT_TRUE(waitForState(StatesEnum::CLEARING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, sm));

  ASSERT_TRUE(sm.reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, sm));

  ASSERT_TRUE(sm.start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETE, sm));

  ASSERT_TRUE(sm.reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, sm));

  ASSERT_TRUE(sm.start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));

  ASSERT_TRUE(sm.hold());
  ASSERT_TRUE(waitForState(StatesEnum::HOLDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::HELD, sm));

  ASSERT_TRUE(sm.unhold());
  ASSERT_TRUE(waitForState(StatesEnum::UNHOLDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));

  ASSERT_TRUE(sm.suspend());
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDED, sm));

  ASSERT_TRUE(sm.unsuspend());
  ASSERT_TRUE(waitForState(StatesEnum::UNSUSPENDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));

  ASSERT_TRUE(sm.stop());
  ASSERT_TRUE(waitForState(StatesEnum::STOPPING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, sm));

  ASSERT_TRUE(sm.abort());
  ASSERT_TRUE(waitForState(StatesEnum::ABORTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, sm));

  sm.deactivate();
  ros::Duration(1).sleep();
  EXPECT_FALSE(sm.isActive());
  ROS_INFO_STREAM("State diagram test complete");
}


TEST(Packml_CC, state_diagram)
{
  ROS_INFO_STREAM("CONTINUOUS CYCLE::State diagram");
  ContinuousCycle sm;
  EXPECT_FALSE(sm.isActive());
  sm.setExecute(std::bind(success));
  sm.activate();
  ros::Duration(1.0).sleep();  //give time to start
  EXPECT_TRUE(sm.isActive());

  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, sm));
  ASSERT_TRUE(sm.isActive());

  ASSERT_TRUE(sm.clear());
  ASSERT_TRUE(waitForState(StatesEnum::CLEARING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, sm));

  ASSERT_TRUE(sm.reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, sm));

  ASSERT_TRUE(sm.start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));
  ASSERT_FALSE(waitForState(StatesEnum::COMPLETING, sm));
  ASSERT_FALSE(waitForState(StatesEnum::COMPLETE, sm));

  ASSERT_TRUE(sm.hold());
  ASSERT_TRUE(waitForState(StatesEnum::HOLDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::HELD, sm));

  ASSERT_TRUE(sm.unhold());
  ASSERT_TRUE(waitForState(StatesEnum::UNHOLDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));

  ASSERT_TRUE(sm.suspend());
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDED, sm));

  ASSERT_TRUE(sm.unsuspend());
  ASSERT_TRUE(waitForState(StatesEnum::UNSUSPENDING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, sm));

  ASSERT_TRUE(sm.stop());
  ASSERT_TRUE(waitForState(StatesEnum::STOPPING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, sm));

  ASSERT_TRUE(sm.abort());
  ASSERT_TRUE(waitForState(StatesEnum::ABORTING, sm));
  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, sm));

  sm.deactivate();
  ros::Duration(1).sleep();
  EXPECT_FALSE(sm.isActive());
  ROS_INFO_STREAM("State diagram test complete");
}



}
