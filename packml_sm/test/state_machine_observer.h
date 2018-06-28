#ifndef STATE_MACHINE_OBSERVER_H
#define STATE_MACHINE_OBSERVER_H

#include "packml_sm/abstract_state_machine.h"

#include <memory>

class StateMachineObserver
{
public:
  StateMachineObserver(std::shared_ptr<packml_sm::AbstractStateMachine> sm);
  ~StateMachineObserver();

  void setStateChangedCallback(std::function<void(int)> callback);

private:
  std::shared_ptr<packml_sm::AbstractStateMachine> sm_;
  std::function<void(int)> state_changed_callback_;

  void handleStateChanged(packml_sm::AbstractStateMachine& state_machine, const packml_sm::StateChangedEventArgs& args);
};

#endif // STATE_MACHINE_OBSERVER_H
