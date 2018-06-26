#pragma once

#include <functional>
#include <memory>

#include "packml_sm/abstract_state_machine.h"

namespace packml_sm
{
class StateMachine : public AbstractStateMachine
{
public:
  bool activate();
  bool deactivate();
  bool setIdle(std::function<int()> state_method);
  bool setStarting(std::function<int()> state_method);
  bool setExecute(std::function<int()> state_method);
  bool setCompleting(std::function<int()> state_method);
  bool setAborting(std::function<int()> state_method);
  bool setClearing(std::function<int()> state_method);
  bool setStopping(std::function<int()> state_method);
  bool setResetting(std::function<int()> state_method);
  bool setSuspending(std::function<int()> state_method);
  bool setUnsuspending(std::function<int()> state_method);
  bool setHolding(std::function<int()> state_method);
  bool setUnholding(std::function<int()> state_method);
  bool isActive();
  int getCurrentState();

protected:
  int state_value_;

  StateMachine();

  void _start() override
  {
  }

  void _clear() override
  {
  }

  void _reset() override
  {
  }

  void _hold() override
  {
  }

  void _unhold() override
  {
  }

  void _suspend() override
  {
  }

  void _unsuspend() override
  {
  }

  void _stop() override
  {
  }

  void _abort() override
  {
  }
};
}
