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
#pragma once

#include <functional>
#include <memory>

#include "packml_sm/common.h"
#include "packml_sm/abstract_state_machine.h"
#include "packml_sm/boost/packml_state_machine.h"

namespace packml_sm
{
class StateMachine : public AbstractStateMachine
{
public:
  StateMachine();
  ~StateMachine();
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
  int state_value_ = static_cast<int>(StatesEnum::ABORTED);

  void _start() override;
  void _clear() override;
  void _reset() override;
  void _hold() override;
  void _unhold() override;
  void _suspend() override;
  void _unsuspend() override;
  void _stop() override;
  void _abort() override;

private:
  boost::msm::back::state_machine<Packml_State_Machine_V3> state_machine_;
  bool is_active_ = false;

  template <typename T>
  bool setStateMethod(std::function<int()> state_method);

  void handleStateChanged(packml_sm::StateChangeNotifier& state_machine, const packml_sm::StateChangedEventArgs& args);
};
}
