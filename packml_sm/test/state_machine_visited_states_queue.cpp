#include "state_machine_visited_states_queue.h"
#include <functional>

StateMachineVisitedStatesQueue::StateMachineVisitedStatesQueue(std::shared_ptr<packml_sm::AbstractStateMachine> sm) : observer_(sm)
{
  observer_.setStateChangedCallback(std::bind(&StateMachineVisitedStatesQueue::stateChanged, this, std::placeholders::_1));
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
