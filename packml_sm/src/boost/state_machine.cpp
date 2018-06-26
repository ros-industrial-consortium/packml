#include "packml_sm/boost/state_machine.h"
#include "packml_sm/boost/packml_events.h"

namespace packml_sm
{
bool StateMachine::activate()
{
  is_active_ = true;
  state_machine_.start();
  return true;
}

bool StateMachine::deactivate()
{
  is_active_ = false;
  state_machine_.stop();
  return true;
}

bool StateMachine::setIdle(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setStarting(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setExecute(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setCompleting(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setAborting(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setClearing(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setStopping(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setResetting(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setSuspending(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setUnsuspending(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setHolding(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::setUnholding(std::function<int()> state_method)
{
  return true;
}

bool StateMachine::isActive()
{
  return is_active_;
}

int StateMachine::getCurrentState()
{
  return state_value_;
}

void StateMachine::_start()
{
  state_machine_.process_event(start_event());
}

void StateMachine::_clear()
{
  state_machine_.process_event(clear_event());
}

void StateMachine::_reset()
{
  state_machine_.process_event(reset_event());
}

void StateMachine::_hold()
{
  state_machine_.process_event(hold_event());
}

void StateMachine::_unhold()
{
  state_machine_.process_event(unhold_event());
}

void StateMachine::_suspend()
{
  state_machine_.process_event(suspend_event());
}

void StateMachine::_unsuspend()
{
  state_machine_.process_event(unsuspend_event());
}

void StateMachine::_stop()
{
  state_machine_.process_event(stop_event());
}

void StateMachine::_abort()
{
  state_machine_.process_event(clear_event());
}
}
