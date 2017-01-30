/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016 Shaun Edwards
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

#include "packml_sm/state_machine.h"

namespace packml_sm
{


//NOTES:
// Create factory methods that take std::bind as an argument for
// a custom call back in the "onExit" method.

// StateMachine will consist of several public SLOTS for each
// PackML command.  The implementations will post events to the SM
// when called.

// Specializations of StateMachine (like ROS StateMachine) will use
// state entered events to trigger status publishing via SLOTS

// Mode handling will be achieved using a hiearchy of state machines
// that reference/utilize many of the same transitions/states (maybe)


StateMachine::StateMachine(BaseState* execute_state)
{
  init(execute_state);
}

StateMachine::StateMachine()
{
  init(ActingState::Execute(NULL));
}

void StateMachine::init(BaseState* execute_state)
{
  ROS_INFO_STREAM("State machine constructor");

  ROS_INFO_STREAM("Forming state machine (states + transitions)");
  ROS_INFO_STREAM("Constructiong super states");
  BaseState* abortable_ = WaitState::Abortable();
  BaseState* stoppable_ = WaitState::Stoppable(abortable_);
  ROS_INFO_STREAM("Constructiong acting/wait states");
  BaseState* unholding_ = ActingState::Unholding(stoppable_);
  BaseState* held_ = WaitState::Held(stoppable_);
  BaseState* holding_ = ActingState::Holding(stoppable_);
  BaseState* idle_ = WaitState::Idle(stoppable_);
  BaseState* starting_ = ActingState::Starting(stoppable_);
  BaseState* completing_ = ActingState::Completing(stoppable_);
  BaseState* complete_ = WaitState::Complete(stoppable_);
  BaseState* resetting_ = ActingState::Resetting(stoppable_);
  BaseState* unsuspending_ = ActingState::Unsuspending(stoppable_);
  BaseState* suspended_ = WaitState::Suspended(stoppable_);
  BaseState* suspending_ = ActingState::Suspending(stoppable_);
  BaseState* stopped_ = WaitState::Stopped(abortable_);
  BaseState* stopping_ = ActingState::Stopping(abortable_);
  BaseState* clearing_ = ActingState::Clearing(abortable_);
  BaseState* aborted_ = WaitState::Aborted();
  BaseState* aborting_ = ActingState::Aborting();

  // special initialization for execute because it is passed in as a method
  // argument
  BaseState* execute_ = execute_state;
  execute_->setParent(stoppable_);

  ROS_INFO_STREAM("Construction and loading transitions");

  //Naming <from state>_<to state>
  //TODO: Still missing some transitions
  CmdTransition* abortable_aborting = CmdTransition::abort(*abortable_, *aborting_);
  StateCompleteTransition* aborting_aborted = new StateCompleteTransition(*aborting_, *aborted_);
  CmdTransition* aborted_clearing_ = CmdTransition::clear(*aborted_, *clearing_);
  StateCompleteTransition* clearing_stopped_ = new StateCompleteTransition(*clearing_, *stopped_);
  CmdTransition* stoppable_stopping_ = CmdTransition::stop(*stoppable_, *stopping_);
  StateCompleteTransition* unholding_execute_ = new StateCompleteTransition(*unholding_, *execute_);
  CmdTransition* held_unholding_ = CmdTransition::unhold(*held_, *unholding_);
  StateCompleteTransition* holding_held_ = new StateCompleteTransition(*holding_,*held_);
  CmdTransition* idle_starting_ = CmdTransition::start(*idle_, *starting_);
  StateCompleteTransition* starting_execute_ = new StateCompleteTransition(*starting_,*execute_);
  CmdTransition* execute_holding_ = CmdTransition::hold(*execute_,*holding_);
  StateCompleteTransition* execute_completing_ = new StateCompleteTransition(*execute_,*completing_);
  CmdTransition* execute_suspending_ = CmdTransition::suspend(*execute_,*suspending_);
  StateCompleteTransition* completing_complete = new StateCompleteTransition(*completing_, *complete_);
  CmdTransition* complete_resetting_ = CmdTransition::reset(*complete_, *resetting_);
  StateCompleteTransition* resetting_idle_ = new StateCompleteTransition(*resetting_, *idle_);
  CmdTransition* suspended_unsuspending_ = CmdTransition::suspend(*suspended_, *unsuspending_);
  StateCompleteTransition* unsuspending_execute_ = new StateCompleteTransition(*unsuspending_, *execute_);


  ROS_INFO_STREAM("Adding states to state machine");
  addState(abortable_);
  addState(aborted_);
  addState(aborting_);
  abortable_->setInitialState(clearing_);
  stoppable_->setInitialState(resetting_);
  setInitialState(aborted_);
  ROS_INFO_STREAM("State machine formed");
}
StateMachine::~StateMachine()
{
}


}
