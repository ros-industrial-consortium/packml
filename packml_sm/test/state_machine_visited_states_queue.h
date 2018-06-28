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

#endif // STATE_MACHINE_VISITED_STATES_QUEUE_H
