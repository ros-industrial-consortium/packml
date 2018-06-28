#include "state_machine_observer.h"
#include "packml_sm/abstract_state_machine.h"

StateMachineObserver::StateMachineObserver(std::shared_ptr<packml_sm::AbstractStateMachine> sm) : sm_(sm)
{
  sm_->stateChangedEvent.bind_member_func(this, &StateMachineObserver::handleStateChanged);
}

StateMachineObserver::~StateMachineObserver()
{
  sm_->stateChangedEvent.unbind_member_func(this, &StateMachineObserver::handleStateChanged);
}

void StateMachineObserver::setStateChangedCallback(std::function<void(int)> callback)
{
  state_changed_callback_ = callback;
}

void StateMachineObserver::handleStateChanged(packml_sm::AbstractStateMachine& state_machine, const packml_sm::StateChangedEventArgs& args)
{
  state_changed_callback_(args.value);
}
