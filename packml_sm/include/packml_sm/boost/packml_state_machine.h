#pragma once

#include "packml_sm/dlog.h"
#include "packml_sm/boost/packml_transition_table.h"

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/back/state_machine.hpp>

namespace packml_sm
{
struct Packml_State_Machine_V3 : public boost::msm::front::state_machine_def<Packml_State_Machine_V3>
{
  typedef Aborted_impl initial_state;

  static boost::msm::back::state_machine<Packml_State_Machine_V3> state_machine_;

  void startStateMachine()
  {
    state_machine_.start();
  }

  void stopStateMachine()
  {
    state_machine_.stop();
  }

  void clear()
  {
    state_machine_.process_event(clear_event());
  }

  void stateComplete()
  {
    state_machine_.process_event(state_complete_event());
  }

  void abort()
  {
    state_machine_.process_event(abort_event());
  }

  void reset()
  {
    state_machine_.process_event(reset_event());
  }

  void stop()
  {
    state_machine_.process_event(stop_event());
  }

  void start()
  {
    state_machine_.process_event(start_event());
  }

  void hold()
  {
    state_machine_.process_event(hold_event());
  }

  void suspend()
  {
    state_machine_.process_event(suspend_event());
  }

  void unhold()
  {
    state_machine_.process_event(unhold_event());
  }

  void unsuspend()
  {
    state_machine_.process_event(unsuspend_event());
  }
};
}
