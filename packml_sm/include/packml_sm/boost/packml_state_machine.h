/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018 Plus One Robotics
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
#pragma once
#include "packml_sm/common.h"
#include "packml_sm/abstract_state_machine.h"
#include "packml_sm/packml_stats_snapshot.h"
#include "packml_sm/boost/packml_states.h"
#include "packml_sm/boost/packml_transitions_continuous.h"
#include "packml_sm/boost/packml_transitions_single_cycle.h"
#include "packml_sm/boost/state_machine_event_loop.h"

#include <map>
#include <functional>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/states.hpp>

namespace packml_sm
{
template <typename T>
class PackmlStateMachine : public AbstractStateMachine
{
public:
  virtual ~PackmlStateMachine();

  // AbstractStateMachine
  virtual bool deactivate() override;
  virtual bool activate() override;
  virtual bool setStarting(std::function<int()> state_method) override;
  virtual bool setExecute(std::function<int()> state_method) override;
  virtual bool setCompleting(std::function<int()> state_method) override;
  virtual bool setAborting(std::function<int()> state_method) override;
  virtual bool setClearing(std::function<int()> state_method) override;
  virtual bool setStopping(std::function<int()> state_method) override;
  virtual bool setResetting(std::function<int()> state_method) override;
  virtual bool setSuspending(std::function<int()> state_method) override;
  virtual bool setUnsuspending(std::function<int()> state_method) override;
  virtual bool setHolding(std::function<int()> state_method) override;
  virtual bool setUnholding(std::function<int()> state_method) override;
  virtual bool isActive() override;

protected:
  PackmlStateMachine();

  virtual void _start() override;
  virtual void _clear() override;
  virtual void _reset() override;
  virtual void _hold() override;
  virtual void _unhold() override;
  virtual void _suspend() override;
  virtual void _unsuspend() override;
  virtual void _stop() override;
  virtual void _abort() override;

private:
  bool is_active_ = false;

  boost::msm::back::state_machine<T> boost_fsm_;
  StateMachineEventLoop event_loop_;

  bool setStateMethod(StatesEnum state, std::function<int()> state_method);
  void sendCommand(CmdEnum command);
  void handleStateChanged(StateChangeNotifier& state_machine, const StateChangedEventArgs& args);
  void update(StateMachineEventLoop& event_loop, const EventArgs& args);
  PackmlState* getPackmlState(StatesEnum state);
};

template class PackmlStateMachine<PackmlTransitionsContinuous>;
template class PackmlStateMachine<PackmlTransitionsSingleCycle>;
}
