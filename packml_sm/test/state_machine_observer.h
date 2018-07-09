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

#endif  // STATE_MACHINE_OBSERVER_H
