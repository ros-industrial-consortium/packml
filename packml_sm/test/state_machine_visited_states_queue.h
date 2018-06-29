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
#ifndef STATE_MACHINE_VISITED_STATES_QUEUE_H
#define STATE_MACHINE_VISITED_STATES_QUEUE_H

#include "state_machine_observer.h"
#include <queue>

class StateMachineVisitedStatesQueue
{
public:
  StateMachineVisitedStatesQueue(std::shared_ptr<packml_sm::AbstractStateMachine> sm);
  bool isEmpty();
  void clear();
  int nextState();

private:
  StateMachineObserver observer_;
  std::queue<int> visited_states_;

  void stateChanged(int new_state);
};

#endif  // STATE_MACHINE_VISITED_STATES_QUEUE_H
