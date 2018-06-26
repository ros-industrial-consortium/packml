#pragma once

#include "packml_sm/dlog.h"

#include <boost/msm/front/state_machine_def.hpp>

namespace packml_sm
{
struct Aborted_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Aborted");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Aborted");
  }
};

struct Clearing_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Clearing");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Clearing");
  }
};

struct Stopped_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Stopped");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Stopped");
  }
};

struct Resetting_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Resetting");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Resetting");
  }
};

struct Idle_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Idle");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Idle");
  }
};

struct Starting_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Starting");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Starting");
  }
};

struct Execute_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Execute");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Execute");
  }
};

struct Holding_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Holding");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Holding");
  }
};

struct Held_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Held");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Held");
  }
};

struct UnHolding_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: UnHolding");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: UnHolding");
  }
};

struct Suspending_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Suspending");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Suspending");
  }
};

struct Suspended_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Suspended");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Suspended");
  }
};

struct UnSuspending_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: UnSuspending");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: UnSuspending");
  }
};

struct Completing_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Completing");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Completing");
  }
};

struct Complete_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Complete");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Complete");
  }
};

struct Aborting_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Aborting");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Aborting");
  }
};

struct Stopping_impl : public boost::msm::front::state<>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    DLog::LogInfo("Entering: Stopping");
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    DLog::LogInfo("Leaving: Stopping");
  }
};
}
