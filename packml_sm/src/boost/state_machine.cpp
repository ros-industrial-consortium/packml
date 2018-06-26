#include "packml_sm/boost/state_machine.h"
#include "packml_sm/boost/packml_events.h"
#include "packml_sm/boost/packml_states.h"
#include "packml_sm/common.h"

#include "packml_sm/event.h"

namespace packml_sm
{
bool StateMachine::activate()
{
  if (!is_active_)
  {
    is_active_ = true;
    state_machine_.start();
    auto state_change_notifier = dynamic_cast<StateChangeNotifier*>(&state_machine_);
    if (state_change_notifier != nullptr)
    {
      state_change_notifier->state_changed_event_.bind_member_func(this, &StateMachine::handleStateChanged);
    }
  }

  return true;
}

bool StateMachine::deactivate()
{
  if (is_active_)
  {
    is_active_ = false;
    state_machine_.stop();
    auto state_change_notifier = dynamic_cast<StateChangeNotifier*>(&state_machine_);
    if (state_change_notifier != nullptr)
    {
      state_change_notifier->state_changed_event_.unbind_member_func(this, &StateMachine::handleStateChanged);
    }
  }

  return true;
}

bool StateMachine::setIdle(std::function<int()> state_method)
{
  return setStateMethod<Idle_impl>(state_method);
}

bool StateMachine::setStarting(std::function<int()> state_method)
{
  return setStateMethod<Starting_impl>(state_method);
}

bool StateMachine::setExecute(std::function<int()> state_method)
{
  return setStateMethod<Execute_impl>(state_method);
}

bool StateMachine::setCompleting(std::function<int()> state_method)
{
  return setStateMethod<Completing_impl>(state_method);
}

bool StateMachine::setAborting(std::function<int()> state_method)
{
  return setStateMethod<Aborting_impl>(state_method);
}

bool StateMachine::setClearing(std::function<int()> state_method)
{
  return setStateMethod<Clearing_impl>(state_method);
}

bool StateMachine::setStopping(std::function<int()> state_method)
{
  return setStateMethod<Stopping_impl>(state_method);
}

bool StateMachine::setResetting(std::function<int()> state_method)
{
  return setStateMethod<Resetting_impl>(state_method);
}

bool StateMachine::setSuspending(std::function<int()> state_method)
{
  return setStateMethod<Suspending_impl>(state_method);
}

bool StateMachine::setUnsuspending(std::function<int()> state_method)
{
  return setStateMethod<UnSuspending_impl>(state_method);
}

bool StateMachine::setHolding(std::function<int()> state_method)
{
  return setStateMethod<Holding_impl>(state_method);
}

bool StateMachine::setUnholding(std::function<int()> state_method)
{
  return setStateMethod<UnHolding_impl>(state_method);
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

template <typename T>
bool StateMachine::setStateMethod(std::function<int()> state_method)
{
  auto current_state = state_machine_.get_state<T*>();
  auto packml_state = dynamic_cast<PackmlState*>(current_state);
  if (packml_state != nullptr)
  {
    packml_state->setStateMethod(state_method);
    return true;
  }

  return false;
}

void StateMachine::handleStateChanged(packml_sm::StateChangeNotifier& state_machine,
                                      const packml_sm::StateChangedEventArgs& args)
{
  state_value_ = args.value;
  invokeStateChangedEvent(args.name, args.value);
}
}
