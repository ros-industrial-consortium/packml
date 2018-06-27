/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018 Joshua Curtis
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
#pragma once

#include "packml_sm/common.h"
#include "packml_sm/dlog.h"
#include "packml_sm/state_change_notifier.h"
#include "packml_sm/boost/packml_events.h"

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/back/state_machine.hpp>

namespace packml_sm
{
struct PackmlState : public boost::msm::front::state<>
{
public:
  virtual std::string stateName() = 0;
  virtual int stateId() = 0;

  void setStateMethod(std::function<int()> state_method)
  {
    state_method_ = state_method;
  }

  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& state_machine)
  {
    auto smi = dynamic_cast<StateChangeNotifier*>(&state_machine);
    if (smi != nullptr)
    {
      smi->handleStateChangeNotify(stateName(), stateId());
    }

    DLog::LogInfo("Entering: %s", stateName().c_str());

    auto result = 0;
    if (state_method_ != nullptr)
    {
      result = state_method_();
    }

    if (result == 0)
    {
      state_machine.process_event(state_complete_event());
    }
    else
    {
      DLog::LogError("Error Processing Event: %s", stateName().c_str());
      state_machine.process_event(error_event());
    }
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& state_machine)
  {
    DLog::LogInfo("Leaving: %s", stateName().c_str());
  }

private:
  std::string state_name_;

  std::function<int()> state_method_;
};

struct Aborted_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Aborted";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::ABORTED);
  }
};

struct Clearing_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Clearing";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::CLEARING);
  }
};

struct Stopped_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Stopped";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::STOPPED);
  }
};

struct Resetting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Resetting";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::RESETTING);
  }
};

struct Idle_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Idle";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::IDLE);
  }
};

struct Starting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Starting";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::STARTING);
  }
};

struct Execute_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Execute";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::EXECUTE);
  }
};

struct Holding_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Holding";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::HOLDING);
  }
};

struct Held_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Held";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::HELD);
  }
};

struct UnHolding_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "UnHolding";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::UNHOLDING);
  }
};

struct Suspending_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Suspending";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::SUSPENDING);
  }
};

struct Suspended_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Suspended";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::SUSPENDED);
  }
};

struct UnSuspending_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "UnSuspending";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::UNSUSPENDING);
  }
};

struct Completing_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Completing";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::COMPLETING);
  }
};

struct Complete_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Complete";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::COMPLETE);
  }
};

struct Aborting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Aborting";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::ABORTING);
  }
};

struct Stopping_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Stopping";
  }

  int stateId()
  {
    return static_cast<int>(StatesEnum::STOPPING);
  }
};
}
