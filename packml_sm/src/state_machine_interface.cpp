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

#include "packml_sm/state_machine_interface.h"
#include "packml_sm/state.h"

namespace packml_sm
{
bool StateMachineInterface::start()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::IDLE:
      _start();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring START command in current state: " << getCurrentState());
      return false;
  }
}

bool StateMachineInterface::clear()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::ABORTED:
      _clear();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring CLEAR command in current state: " << getCurrentState());
      return false;
  }
}

bool StateMachineInterface::reset()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::COMPLETE:
    case StatesEnum::STOPPED:
      _reset();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring RESET command in current state: " << getCurrentState());
      return false;
  }
}

bool StateMachineInterface::hold()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::EXECUTE:
      _hold();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring HOLD command in current state: " << getCurrentState());
      return false;
  }
}

bool StateMachineInterface::unhold()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::HELD:
      _unhold();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring HELD command in current state: " << getCurrentState());
      return false;
  }
}

bool StateMachineInterface::suspend()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::EXECUTE:
      _suspend();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring SUSPEND command in current state: " << getCurrentState());
      return false;
  }
}

bool StateMachineInterface::unsuspend()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::SUSPENDED:
      _unsuspend();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring UNSUSPEND command in current state: " << getCurrentState());
      return false;
  }
}

bool StateMachineInterface::stop()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::STOPPABLE:
    case StatesEnum::STARTING:
    case StatesEnum::IDLE:
    case StatesEnum::SUSPENDED:
    case StatesEnum::EXECUTE:
    case StatesEnum::HOLDING:
    case StatesEnum::HELD:
    case StatesEnum::SUSPENDING:
    case StatesEnum::UNSUSPENDING:
    case StatesEnum::UNHOLDING:
    case StatesEnum::COMPLETING:
    case StatesEnum::COMPLETE:
      _stop();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring STOP command in current state: " << getCurrentState());
      return false;
  }
}

bool StateMachineInterface::abort()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::ABORTABLE:
    case StatesEnum::STOPPED:
    case StatesEnum::STARTING:
    case StatesEnum::IDLE:
    case StatesEnum::SUSPENDED:
    case StatesEnum::EXECUTE:
    case StatesEnum::HOLDING:
    case StatesEnum::HELD:
    case StatesEnum::SUSPENDING:
    case StatesEnum::UNSUSPENDING:
    case StatesEnum::UNHOLDING:
    case StatesEnum::COMPLETING:
    case StatesEnum::COMPLETE:
    case StatesEnum::CLEARING:
    case StatesEnum::STOPPING:
      _abort();
      return true;
    default:
      ROS_WARN_STREAM("Ignoring ABORT command in current state: " << getCurrentState());
      return false;
  }
}
}
