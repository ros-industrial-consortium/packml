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
#include "packml_sm/transitions.h"

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


StateMachine::StateMachine(PackmlState* execute_state)
{
  init(execute_state);
}

StateMachine::StateMachine()
{
  init(ActingState::Execute(NULL));
}

void StateMachine::init(PackmlState* execute_state)
{
  ROS_INFO_STREAM("State machine constructor");

  ROS_INFO_STREAM("Forming state machine (states + transitions)");
  ROS_INFO_STREAM("Constructiong super states");
  PackmlState* abortable_ = WaitState::Abortable();
  connect(abortable_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* stoppable_ = WaitState::Stoppable(abortable_);
  connect(stoppable_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  ROS_INFO_STREAM("Constructiong acting/wait states");
  PackmlState* unholding_ = ActingState::Unholding(stoppable_);
  connect(unholding_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* held_ = WaitState::Held(stoppable_);
  connect(held_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* holding_ = ActingState::Holding(stoppable_);
  connect(holding_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* idle_ = WaitState::Idle(stoppable_);
  connect(idle_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* starting_ = ActingState::Starting(stoppable_);
  connect(starting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* completing_ = ActingState::Completing(stoppable_);
  connect(completing_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* complete_ = WaitState::Complete(stoppable_);
  connect(complete_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* resetting_ = ActingState::Resetting(stoppable_);
  connect(resetting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* unsuspending_ = ActingState::Unsuspending(stoppable_);
  connect(unsuspending_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* suspended_ = WaitState::Suspended(stoppable_);
  connect(suspended_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* suspending_ = ActingState::Suspending(stoppable_);
  connect(suspending_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* stopped_ = WaitState::Stopped(abortable_);
  connect(stopped_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* stopping_ = ActingState::Stopping(abortable_);
  connect(stopping_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* clearing_ = ActingState::Clearing(abortable_);
  connect(clearing_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* aborted_ = WaitState::Aborted();
  connect(aborted_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  PackmlState* aborting_ = ActingState::Aborting();
  connect(aborting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  // special initialization for execute because it is passed in as a method
  // argument
  PackmlState* execute_ = execute_state;
  connect(execute_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  execute_->setParent(stoppable_);

  ROS_INFO_STREAM("Construction and loading transitions");

  //Naming <from state>_<to state>
  CmdTransition*                abortable_aborting = CmdTransition::abort(*abortable_, *aborting_);
  StateCompleteTransition*      aborting_aborted = new StateCompleteTransition(*aborting_, *aborted_);
  CmdTransition*                aborted_clearing_ = CmdTransition::clear(*aborted_, *clearing_);
  StateCompleteTransition*      clearing_stopped_ = new StateCompleteTransition(*clearing_, *stopped_);
  CmdTransition*                stoppable_stopping_ = CmdTransition::stop(*stoppable_, *stopping_);
  StateCompleteTransition*      stopping_stopped = new StateCompleteTransition(*stopping_, *stopped_);
  CmdTransition*                stopped_resetting_ = CmdTransition::reset(*stopped_, *resetting_);
  StateCompleteTransition*      unholding_execute_ = new StateCompleteTransition(*unholding_, *execute_);
  CmdTransition*                held_unholding_ = CmdTransition::unhold(*held_, *unholding_);
  StateCompleteTransition*      holding_held_ = new StateCompleteTransition(*holding_,*held_);
  CmdTransition*                idle_starting_ = CmdTransition::start(*idle_, *starting_);
  StateCompleteTransition*      starting_execute_ = new StateCompleteTransition(*starting_,*execute_);
  CmdTransition*                execute_holding_ = CmdTransition::hold(*execute_,*holding_);
  StateCompleteTransition*      execute_completing_ = new StateCompleteTransition(*execute_,*completing_);
  StateCompleteTransition*      completing_complete = new StateCompleteTransition(*completing_, *complete_);
  CmdTransition*                complete_resetting_ = CmdTransition::reset(*complete_, *resetting_);
  StateCompleteTransition*      resetting_idle_ = new StateCompleteTransition(*resetting_, *idle_);
  CmdTransition*                execute_suspending_ = CmdTransition::suspend(*execute_,*suspending_);
  StateCompleteTransition*      suspending_suspended_ = new StateCompleteTransition(*suspending_,*suspended_);
  CmdTransition*                suspended_unsuspending_ = CmdTransition::unsuspend(*suspended_, *unsuspending_);
  StateCompleteTransition*      unsuspending_execute_ = new StateCompleteTransition(*unsuspending_, *execute_);


  ROS_INFO_STREAM("Adding states to state machine");
  addState(abortable_);
  addState(aborted_);
  addState(aborting_);
  abortable_->setInitialState(clearing_);
  stoppable_->setInitialState(resetting_);
  setInitialState(aborted_);
  ROS_INFO_STREAM("State machine formed");
}

void StateMachine::setState(int value, QString name)
{
  ROS_INFO_STREAM("State changed(event) to: " << name.toStdString() <<
                  "(" << value << ")");
  state_value_ = value;
  state_name_ = name;
}


}
