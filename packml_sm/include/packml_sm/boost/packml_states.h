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
#pragma once

#include "packml_sm/common.h"
#include "packml_sm/dlog.h"
#include "packml_sm/state_change_notifier.h"
#include "packml_sm/boost/packml_events.h"

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <future>

namespace packml_sm
{
struct PackmlState : public boost::msm::front::state<>
{
public:
  virtual std::string stateName() = 0;
  virtual StatesEnum stateId() = 0;

  void setStateMethod(std::function<int()> state_method)
  {
    state_method_ = state_method;
  }

  template <class FSM>
  void runStateMethod(FSM* state_machine_ptr)
  {
    if (state_method_ != nullptr)
    {
      auto result = state_method_();
      if (result == 0)
      {
        if (!is_exiting_)
        {
          state_machine_ptr->enqueue_event(state_complete_event());
        }
      }
      else
      {
        DLog::LogInfo("Error running state method: %s", state_name_.c_str());
        state_machine_ptr->enqueue_event(error_event());
      }
    }
    else
    {
      state_machine_ptr->enqueue_event(state_complete_event());
    }
  }

  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& state_machine)
  {
    start_time_ = std::chrono::steady_clock::now();

    is_exiting_ = false;
    is_running_ = true;
    auto smi = dynamic_cast<StateChangeNotifier*>(&state_machine);
    if (smi != nullptr)
    {
      smi->handleStateChangeNotify(stateName(), stateId());
    }

    DLog::LogInfo("Entering: %s", stateName().c_str());

    state_method_future_ = std::async(
        std::launch::async, std::bind(&PackmlState::runStateMethod<FSM>, this, std::placeholders::_1), &state_machine);
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& state_machine)
  {
    is_exiting_ = true;
    if (state_method_future_.valid())
    {
      state_method_future_.get();
    }

    is_running_ = false;
    DLog::LogInfo("Leaving: %s", stateName().c_str());
  }

private:
  std::atomic<bool> is_running_;
  std::atomic<bool> is_exiting_;
  std::string state_name_;
  std::function<int()> state_method_;
  std::future<void> state_method_future_;
  std::chrono::steady_clock::time_point start_time_;
  double cummulative_time_ = 0.0f;
};

struct Aborted_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Aborted";
  }

  StatesEnum stateId()
  {
    return StatesEnum::ABORTED;
  }
};

struct Clearing_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Clearing";
  }

  StatesEnum stateId()
  {
    return StatesEnum::CLEARING;
  }
};

struct Stopped_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Stopped";
  }

  StatesEnum stateId()
  {
    return StatesEnum::STOPPED;
  }
};

struct Resetting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Resetting";
  }

  StatesEnum stateId()
  {
    return StatesEnum::RESETTING;
  }
};

struct Idle_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Idle";
  }

  StatesEnum stateId()
  {
    return StatesEnum::IDLE;
  }
};

struct Starting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Starting";
  }

  StatesEnum stateId()
  {
    return StatesEnum::STARTING;
  }
};

struct Execute_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Execute";
  }

  StatesEnum stateId()
  {
    return StatesEnum::EXECUTE;
  }
};

struct Holding_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Holding";
  }

  StatesEnum stateId()
  {
    return StatesEnum::HOLDING;
  }
};

struct Held_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Held";
  }

  StatesEnum stateId()
  {
    return StatesEnum::HELD;
  }
};

struct UnHolding_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "UnHolding";
  }

  StatesEnum stateId()
  {
    return StatesEnum::UNHOLDING;
  }
};

struct Suspending_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Suspending";
  }

  StatesEnum stateId()
  {
    return StatesEnum::SUSPENDING;
  }
};

struct Suspended_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Suspended";
  }

  StatesEnum stateId()
  {
    return StatesEnum::SUSPENDED;
  }
};

struct UnSuspending_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "UnSuspending";
  }

  StatesEnum stateId()
  {
    return StatesEnum::UNSUSPENDING;
  }
};

struct Completing_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Completing";
  }

  StatesEnum stateId()
  {
    return StatesEnum::COMPLETING;
  }
};

struct Complete_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Complete";
  }

  StatesEnum stateId()
  {
    return StatesEnum::COMPLETE;
  }
};

struct Aborting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Aborting";
  }

  StatesEnum stateId()
  {
    return StatesEnum::ABORTING;
  }
};

struct Stopping_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Stopping";
  }

  StatesEnum stateId()
  {
    return StatesEnum::STOPPING;
  }
};
}
