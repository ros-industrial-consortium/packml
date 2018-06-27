/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018 Joshua Curtis
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "packml_sm/boost/packml_state_machine.h"
#include "packml_sm/boost/packml_state_machine_continuous.h"

namespace packml_sm
{
template <typename T>
PackmlStateMachine<T>::PackmlStateMachine()
{
  auto state_change_notifier = dynamic_cast<StateChangeNotifier*>(&boost_fsm_);
  if (state_change_notifier != nullptr)
  {
    state_change_notifier->stateChangedEvent.bind_member_func(this, &PackmlStateMachine<T>::handleStateChanged);
  }
}

template <typename T>
PackmlStateMachine<T>::~PackmlStateMachine()
{
  auto state_change_notifier = dynamic_cast<StateChangeNotifier*>(&boost_fsm_);
  if (state_change_notifier != nullptr)
  {
    state_change_notifier->stateChangedEvent.unbind_member_func(this, &PackmlStateMachine<T>::handleStateChanged);
  }
}

template <typename T>
bool PackmlStateMachine<T>::activate()
{
  boost_fsm_.start();
  return true;
}

template <typename T>
bool PackmlStateMachine<T>::setStarting(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::STARTING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setExecute(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::EXECUTE, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setCompleting(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::COMPLETING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setAborting(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::ABORTING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setClearing(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::CLEARING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setStopping(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::STOPPING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setResetting(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::RESETTING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setSuspending(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::SUSPENDING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setUnsuspending(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::UNSUSPENDING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setHolding(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::HOLDING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::setUnholding(std::function<int()> state_method)
{
  setStateMethod(StatesEnum::UNHOLDING, state_method);
}

template <typename T>
bool PackmlStateMachine<T>::isActive()
{
  // JJC TODO:
  return true;
}

template <typename T>
int PackmlStateMachine<T>::getCurrentState()
{
  return current_state_;
}

template <typename T>
void PackmlStateMachine<T>::_start()
{
  sendCommand(CmdEnum::START);
}

template <typename T>
void PackmlStateMachine<T>::_clear()
{
  sendCommand(CmdEnum::CLEAR);
}

template <typename T>
void PackmlStateMachine<T>::_reset()
{
  sendCommand(CmdEnum::RESET);
}

template <typename T>
void PackmlStateMachine<T>::_hold()
{
  sendCommand(CmdEnum::HOLD);
}

template <typename T>
void PackmlStateMachine<T>::_unhold()
{
  sendCommand(CmdEnum::UNHOLD);
}

template <typename T>
void PackmlStateMachine<T>::_suspend()
{
  sendCommand(CmdEnum::SUSPEND);
}

template <typename T>
void PackmlStateMachine<T>::_unsuspend()
{
  sendCommand(CmdEnum::UNSUSPEND);
}

template <typename T>
void PackmlStateMachine<T>::_stop()
{
  sendCommand(CmdEnum::STOP);
}

template <typename T>
void PackmlStateMachine<T>::_abort()
{
  sendCommand(CmdEnum::ABORT);
}

template <typename T>
void PackmlStateMachine<T>::sendCommand(CmdEnum command)
{
  switch (command)
  {
    case CmdEnum::CLEAR:
      boost_fsm_.process_event(clear_event());
      break;
    case CmdEnum::START:
      boost_fsm_.process_event(start_event());
      break;
    case CmdEnum::STOP:
      boost_fsm_.process_event(stop_event());
      break;
    case CmdEnum::HOLD:
      boost_fsm_.process_event(hold_event());
      break;
    case CmdEnum::ABORT:
      boost_fsm_.process_event(abort_event());
      break;
    case CmdEnum::RESET:
      boost_fsm_.process_event(reset_event());
      break;
    case CmdEnum::SUSPEND:
      boost_fsm_.process_event(suspend_event());
      break;
    case CmdEnum::UNSUSPEND:
      boost_fsm_.process_event(unsuspend_event());
      break;
    case CmdEnum::UNHOLD:
      boost_fsm_.process_event(unhold_event());
      break;
    default:
      DLog::LogError("Unsupported command requested.");
  }
}

template <typename T>
bool PackmlStateMachine<T>::setStateMethod(StatesEnum state, std::function<int()> state_method)
{
  PackmlState* state_machine_state = nullptr;
  switch (state)
  {
    case StatesEnum::STOPPED:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Stopped_impl*>());
      break;
    case StatesEnum::STARTING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Starting_impl*>());
      break;
    case StatesEnum::IDLE:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Idle_impl*>());
      break;
    case StatesEnum::SUSPENDED:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Suspended_impl*>());
      break;
    case StatesEnum::EXECUTE:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Execute_impl*>());
      break;
    case StatesEnum::STOPPING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Stopping_impl*>());
      break;
    case StatesEnum::ABORTING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Aborting_impl*>());
      break;
    case StatesEnum::ABORTED:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Aborted_impl*>());
      break;
    case StatesEnum::HOLDING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Holding_impl*>());
      break;
    case StatesEnum::HELD:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Held_impl*>());
      break;
    case StatesEnum::RESETTING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Resetting_impl*>());
      break;
    case StatesEnum::SUSPENDING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Suspending_impl*>());
      break;
    case StatesEnum::UNSUSPENDING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<UnSuspending_impl*>());
      break;
    case StatesEnum::CLEARING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Clearing_impl*>());
      break;
    case StatesEnum::UNHOLDING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Stopped_impl*>());
      break;
    case StatesEnum::COMPLETING:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Completing_impl*>());
      break;
    case StatesEnum::COMPLETE:
      state_machine_state = static_cast<PackmlState*>(boost_fsm_.template get_state<Complete_impl*>());
      break;
    default:
      DLog::LogError("Invalid state for setting state method");
  }

  if (state_machine_state != nullptr)
  {
    state_machine_state->setStateMethod(state_method);
    return true;
  }

  return false;
}

template <typename T>
void PackmlStateMachine<T>::handleStateChanged(packml_sm::StateChangeNotifier& state_machine,
                                               const packml_sm::StateChangedEventArgs& args)
{
  current_state_ = args.value;
  invokeStateChangedEvent(args.name, args.value);
}
}
