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

void StateMachineObserver::handleStateChanged(packml_sm::AbstractStateMachine& state_machine,
                                              const packml_sm::StateChangedEventArgs& args)
{
  state_changed_callback_(static_cast<int>(args.value));
}
