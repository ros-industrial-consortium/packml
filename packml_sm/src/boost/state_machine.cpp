#include "packml_sm/boost/state_machine.h"

namespace packml_sm
{
bool StateMachine::activate()
{
  return true;
}

bool StateMachine::deactivate()
{
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
  return true;
}

int StateMachine::getCurrentState()
{
  return state_value_;
}
}
