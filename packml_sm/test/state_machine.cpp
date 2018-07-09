/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018 Plus One Robotics
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
#include "state_machine_visited_states_queue.h"
#include "packml_sm/common.h"
#include "packml_sm/abstract_state_machine.h"
#include "packml_sm/boost/packml_state_machine_single_cycle.h"
#include "packml_sm/boost/packml_state_machine_continuous.h"
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
bool waitForState(StatesEnum state, StateMachineVisitedStatesQueue& queue)
{
  auto i = 0;
  do
  {
    auto nextState = queue.nextState();

    if (nextState == static_cast<int>(state))
    {
      return true;
    }
    else if (nextState != -1)
    {
      return false;
    }

    ROS_WARN_STREAM("Waiting for state to change to " << state);
    ros::Duration(0.05).sleep();
    i++;
  } while (i < 25);

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
  std::shared_ptr<AbstractStateMachine> sm = PackmlStateMachineSingleCycle::spawn();
  StateMachineVisitedStatesQueue queue(sm);
  sm->setExecute(std::bind(success));
  sm->activate();
  ros::Duration(1.0).sleep();  // give time to start
  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, queue));
  ASSERT_TRUE(sm->clear());
  ASSERT_TRUE(waitForState(StatesEnum::CLEARING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, queue));
  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, queue));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETE, queue));
  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, queue));
  sm->setExecute(std::bind(fail));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));
  ASSERT_TRUE(waitForState(StatesEnum::ABORTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, queue));
}

TEST(Packml_SC, state_diagram)
{
  ROS_INFO_STREAM("SINGLE CYCLE::State diagram");
  std::shared_ptr<AbstractStateMachine> sm = PackmlStateMachineSingleCycle::spawn();
  StateMachineVisitedStatesQueue queue(sm);

  EXPECT_FALSE(sm->isActive());
  sm->setExecute(std::bind(success));
  sm->activate();
  ros::Duration(1.0).sleep();  // give time to start
  EXPECT_TRUE(sm->isActive());

  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, queue));
  ASSERT_TRUE(sm->isActive());

  ASSERT_TRUE(sm->clear());
  ASSERT_TRUE(waitForState(StatesEnum::CLEARING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, queue));

  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, queue));

  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::COMPLETE, queue));

  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, queue));

  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));
  ASSERT_TRUE(sm->hold());
  ASSERT_TRUE(waitForState(StatesEnum::HOLDING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::HELD, queue));
  ASSERT_TRUE(sm->unhold());
  ASSERT_TRUE(waitForState(StatesEnum::UNHOLDING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));

  ASSERT_TRUE(sm->suspend());
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDED, queue));
  ASSERT_TRUE(sm->unsuspend());
  ASSERT_TRUE(waitForState(StatesEnum::UNSUSPENDING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));

  ASSERT_TRUE(sm->stop());
  ASSERT_TRUE(waitForState(StatesEnum::STOPPING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, queue));

  ASSERT_TRUE(sm->abort());
  ASSERT_TRUE(waitForState(StatesEnum::ABORTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, queue));

  sm->deactivate();
  ros::Duration(1).sleep();
  EXPECT_FALSE(sm->isActive());
  ROS_INFO_STREAM("State diagram test complete");
}

TEST(Packml_CC, state_diagram)
{
  ROS_INFO_STREAM("CONTINUOUS CYCLE::State diagram");
  std::shared_ptr<AbstractStateMachine> sm = PackmlStateMachineContinuous::spawn();
  StateMachineVisitedStatesQueue queue(sm);
  EXPECT_FALSE(sm->isActive());
  sm->setExecute(std::bind(success));
  sm->activate();
  ros::Duration(1.0).sleep();  // give time to start
  EXPECT_TRUE(sm->isActive());

  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, queue));
  ASSERT_TRUE(sm->isActive());

  ASSERT_TRUE(sm->clear());
  ASSERT_TRUE(waitForState(StatesEnum::CLEARING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, queue));

  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(waitForState(StatesEnum::RESETTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::IDLE, queue));

  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(waitForState(StatesEnum::STARTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));
  ASSERT_FALSE(waitForState(StatesEnum::COMPLETING, queue));
  ASSERT_FALSE(waitForState(StatesEnum::COMPLETE, queue));

  ASSERT_TRUE(sm->hold());
  ASSERT_TRUE(waitForState(StatesEnum::HOLDING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::HELD, queue));

  ASSERT_TRUE(sm->unhold());
  ASSERT_TRUE(waitForState(StatesEnum::UNHOLDING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));

  ASSERT_TRUE(sm->suspend());
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::SUSPENDED, queue));

  ASSERT_TRUE(sm->unsuspend());
  ASSERT_TRUE(waitForState(StatesEnum::UNSUSPENDING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::EXECUTE, queue));

  ASSERT_TRUE(sm->stop());
  ASSERT_TRUE(waitForState(StatesEnum::STOPPING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::STOPPED, queue));

  ASSERT_TRUE(sm->abort());
  ASSERT_TRUE(waitForState(StatesEnum::ABORTING, queue));
  ASSERT_TRUE(waitForState(StatesEnum::ABORTED, queue));

  sm->deactivate();
  ros::Duration(1).sleep();
  EXPECT_FALSE(sm->isActive());
  ROS_INFO_STREAM("State diagram test complete");
}
}
