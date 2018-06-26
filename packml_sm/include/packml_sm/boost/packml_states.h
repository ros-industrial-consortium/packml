#pragma once

#include "packml_sm/boost/packml_events.h"
#include "packml_sm/dlog.h"
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/back/state_machine.hpp>

namespace packml_sm
{
struct PackmlState : public boost::msm::front::state<>
{
public:
  PackmlState(const std::string& state_name)
  {
  }

  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& state_machine)
  {
    DLog::LogInfo("Entering: %s", state_name_.c_str());
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& state_machine)
  {
    DLog::LogInfo("Leaving: %s", state_name_.c_str());
  }

private:
  std::string state_name_;
};

struct Aborted_impl : public PackmlState
{
public:
  Aborted_impl() : PackmlState("Aborted")
  {
  }
};

struct Clearing_impl : public PackmlState
{
public:
  Clearing_impl() : PackmlState("Clearing")
  {
  }
};

struct Stopped_impl : public PackmlState
{
public:
  Stopped_impl() : PackmlState("Stopped")
  {
  }
};

struct Resetting_impl : public PackmlState
{
public:
  Resetting_impl() : PackmlState("Resetting")
  {
  }
};

struct Idle_impl : public PackmlState
{
public:
  Idle_impl() : PackmlState("Idle")
  {
  }
};

struct Starting_impl : public PackmlState
{
public:
  Starting_impl() : PackmlState("Idle")
  {
  }
};

struct Execute_impl : public PackmlState
{
public:
  Execute_impl() : PackmlState("Execute")
  {
  }
};

struct Holding_impl : public PackmlState
{
public:
  Holding_impl() : PackmlState("Holding")
  {
  }
};

struct Held_impl : public PackmlState
{
public:
  Held_impl() : PackmlState("Held")
  {
  }
};

struct UnHolding_impl : public PackmlState
{
  UnHolding_impl() : PackmlState("UnHolding")
  {
  }
};

struct Suspending_impl : public PackmlState
{
  Suspending_impl() : PackmlState("Suspending")
  {
  }
};

struct Suspended_impl : public PackmlState
{
  Suspended_impl() : PackmlState("Suspended")
  {
  }
};

struct UnSuspending_impl : public PackmlState
{
  UnSuspending_impl() : PackmlState("UnSuspending")
  {
  }
};

struct Completing_impl : public PackmlState
{
  Completing_impl() : PackmlState("Completing")
  {
  }
};

struct Complete_impl : public PackmlState
{
  Complete_impl() : PackmlState("Complete")
  {
  }
};

struct Aborting_impl : public PackmlState
{
  Aborting_impl() : PackmlState("Aborting")
  {
  }
};

struct Stopping_impl : public PackmlState
{
  Stopping_impl() : PackmlState("Stopping")
  {
  }
};
}
