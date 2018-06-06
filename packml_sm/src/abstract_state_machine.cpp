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

#include "packml_sm/abstract_state_machine.h"
#include "packml_sm/state.h"
#include "packml_sm/dlog.h"

namespace packml_sm
{
bool AbstractStateMachine::start()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::IDLE:
      _start();
      return true;
    default:
      DLog::LogWarning("Ignoring START command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::clear()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::ABORTED:
      _clear();
      return true;
    default:
      DLog::LogWarning("Ignoring CLEAR command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::reset()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::COMPLETE:
    case StatesEnum::STOPPED:
      _reset();
      return true;
    default:
      DLog::LogWarning("Ignoring RESET command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::hold()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::EXECUTE:
      _hold();
      return true;
    default:
      DLog::LogWarning("Ignoring HOLD command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::unhold()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::HELD:
      _unhold();
      return true;
    default:
      DLog::LogWarning("Ignoring HELD command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::suspend()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::EXECUTE:
      _suspend();
      return true;
    default:
      DLog::LogWarning("Ignoring SUSPEND command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::unsuspend()
{
  switch (StatesEnum(getCurrentState()))
  {
    case StatesEnum::SUSPENDED:
      _unsuspend();
      return true;
    default:
      DLog::LogWarning("Ignoring UNSUSPEND command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::stop()
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
      DLog::LogWarning("Ignoring STOP command in current state: %d", getCurrentState());
      return false;
  }
}

bool AbstractStateMachine::abort()
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
      DLog::LogWarning("Ignoring ABORT command in current state: %d", getCurrentState());
      return false;
  }
}

void AbstractStateMachine::invokeStateChangedEvent(const std::string& name, int value)
{
  state_changed_event_.invoke(*this, { name, value });
}
}
