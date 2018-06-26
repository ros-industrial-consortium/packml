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
  virtual std::string stateName() = 0;

  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& state_machine)
  {
    DLog::LogInfo("Entering: %s", stateName().c_str());
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& state_machine)
  {
    DLog::LogInfo("Leaving: %s", stateName().c_str());
  }

private:
  std::string state_name_;
};

struct Aborted_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Aborted";
  }
};

struct Clearing_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Clearing";
  }
};

struct Stopped_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Stopped";
  }
};

struct Resetting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Resetting";
  }
};

struct Idle_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Idle";
  }
};

struct Starting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Starting";
  }
};

struct Execute_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Execute";
  }
};

struct Holding_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Holding";
  }
};

struct Held_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Held";
  }
};

struct UnHolding_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "UnHolding";
  }
};

struct Suspending_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Suspending";
  }
};

struct Suspended_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Suspended";
  }
};

struct UnSuspending_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "UnSuspending";
  }
};

struct Completing_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Completing";
  }
};

struct Complete_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Complete";
  }
};

struct Aborting_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Aborting";
  }
};

struct Stopping_impl : public PackmlState
{
public:
  std::string stateName()
  {
    return "Stopping";
  }
};
}
