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
#include "state_machine_visited_states_queue.h"
#include <functional>

StateMachineVisitedStatesQueue::StateMachineVisitedStatesQueue(std::shared_ptr<packml_sm::AbstractStateMachine> sm)
  : observer_(sm)
{
  observer_.setStateChangedCallback(
      std::bind(&StateMachineVisitedStatesQueue::stateChanged, this, std::placeholders::_1));
}

bool StateMachineVisitedStatesQueue::isEmpty()
{
  return visited_states_.size() == 0;
}

void StateMachineVisitedStatesQueue::clear()
{
  visited_states_.empty();
}

int StateMachineVisitedStatesQueue::nextState()
{
  auto result = -1;
  if (visited_states_.size() > 0)
  {
    result = visited_states_.front();
    visited_states_.pop();
  }

  return result;
}

void StateMachineVisitedStatesQueue::stateChanged(int new_state)
{
  visited_states_.push(new_state);
}
