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
#include "packml_sm/events.h"

namespace packml_sm
{


bool StateMachineInterface::start()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::IDLE:
    _start();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring START command in current state: " << getCurrentState());
    return false;
  }
}

bool StateMachineInterface::clear()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::ABORTED:
    _clear();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring CLEAR command in current state: " << getCurrentState());
    return false;
  }
}

bool StateMachineInterface::reset()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::COMPLETE:
  case StatesEnum::STOPPED:
    _reset();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring RESET command in current state: " << getCurrentState());
    return false;
  }
}

bool StateMachineInterface::hold()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::EXECUTE:
    _hold();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring HOLD command in current state: " << getCurrentState());
    return false;
  }
}


bool StateMachineInterface::unhold()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::HELD:
    _unhold();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring HELD command in current state: " << getCurrentState());
    return false;
  }
}


bool StateMachineInterface::suspend()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::EXECUTE:
    _suspend();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring SUSPEND command in current state: " << getCurrentState());
    return false;
  }
}

bool StateMachineInterface::unsuspend()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::SUSPENDED:
    _unsuspend();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring UNSUSPEND command in current state: " << getCurrentState());
    return false;
  }
}


bool StateMachineInterface::stop()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::STOPPABLE:
  case StatesEnum::STARTING:
  case StatesEnum::IDLE:
  case StatesEnum::SUSPENDED:
  case StatesEnum::EXECUTE:
  case StatesEnum::HOLDING:
  case StatesEnum::HELD:
  case StatesEnum::SUSPENDING:
  case StatesEnum::UNSUSPENDING:
  case StatesEnum::UNHOLDING:
  case StatesEnum::COMPLETING:
  case StatesEnum::COMPLETE:
    _stop();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring STOP command in current state: " << getCurrentState());
    return false;
  }
}

bool StateMachineInterface::abort()
{
  switch(StatesEnum(getCurrentState())) {
  case StatesEnum::ABORTABLE:
  case StatesEnum::STOPPED:
  case StatesEnum::STARTING:
  case StatesEnum::IDLE:
  case StatesEnum::SUSPENDED:
  case StatesEnum::EXECUTE:
  case StatesEnum::HOLDING:
  case StatesEnum::HELD:
  case StatesEnum::SUSPENDING:
  case StatesEnum::UNSUSPENDING:
  case StatesEnum::UNHOLDING:
  case StatesEnum::COMPLETING:
  case StatesEnum::COMPLETE:
  case StatesEnum::CLEARING:
  case StatesEnum::STOPPING:
    _abort();
    return true;
  default:
    ROS_WARN_STREAM("Ignoring ABORT command in current state: " << getCurrentState());
    return false;
  }
}





QCoreApplication *a;
void init(int argc, char *argv[])
{
  if( NULL == QCoreApplication::instance() )
  {
    ROS_INFO_STREAM("Starting QCoreApplication");
    a = new QCoreApplication(argc, argv);
  }
}






std::shared_ptr<StateMachine> StateMachine::singleCyleSM()
{
  return std::shared_ptr<StateMachine>(new SingleCycle());
}


std::shared_ptr<StateMachine> StateMachine::continuousCycleSM()
{
  return std::shared_ptr<StateMachine>(new ContinuousCycle());
}


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


StateMachine::StateMachine()
{

  ROS_DEBUG_STREAM("State machine constructor");

  ROS_DEBUG_STREAM("Constructiong super states");
  abortable_ = WaitState::Abortable();
  connect(abortable_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  stoppable_ = WaitState::Stoppable(abortable_);
  connect(stoppable_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  ROS_DEBUG_STREAM("Constructiong acting/wait states");
  unholding_ = ActingState::Unholding(stoppable_);
  connect(unholding_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  held_ = WaitState::Held(stoppable_);
  connect(held_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  holding_ = ActingState::Holding(stoppable_);
  connect(holding_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  idle_ = WaitState::Idle(stoppable_);
  connect(idle_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  starting_ = ActingState::Starting(stoppable_);
  connect(starting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  completing_ = ActingState::Completing(stoppable_);
  connect(completing_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  complete_ = WaitState::Complete(stoppable_);
  connect(complete_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  resetting_ = ActingState::Resetting(stoppable_);
  connect(resetting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  unsuspending_ = ActingState::Unsuspending(stoppable_);
  connect(unsuspending_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  suspended_ = WaitState::Suspended(stoppable_);
  connect(suspended_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  suspending_ = ActingState::Suspending(stoppable_);
  connect(suspending_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  stopped_ = WaitState::Stopped(abortable_);
  connect(stopped_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  stopping_ = ActingState::Stopping(abortable_);
  connect(stopping_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  clearing_ = ActingState::Clearing(abortable_);
  connect(clearing_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  aborted_ = WaitState::Aborted();
  connect(aborted_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  aborting_ = ActingState::Aborting();
  connect(aborting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  execute_ = ActingState::Execute(stoppable_);
  connect(execute_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int,QString)));

  ROS_DEBUG_STREAM("Adding states to state machine");
  sm_internal_.addState(abortable_);
  sm_internal_.addState(aborted_);
  sm_internal_.addState(aborting_);
}

StateMachine::~StateMachine()
{
}


bool StateMachine::activate()
{
  ROS_INFO_STREAM("Checking if QCore application is running");
  if( NULL == QCoreApplication::instance() )
  {
    ROS_ERROR_STREAM("QCore application is not running, QCoreApplication must"
                    << " be created in main thread for state macine to run");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("Moving state machine to Qcore thread");
    sm_internal_.moveToThread(QCoreApplication::instance()->thread());
    this->moveToThread(QCoreApplication::instance()->thread());

    sm_internal_.start();
    ROS_INFO_STREAM("State machine thread created and started");

    return true;
  }
}


bool StateMachine::deactivate()
{
  ROS_DEBUG_STREAM("Deactivating state machine");
  sm_internal_.stop();
}

void StateMachine::setState(int value, QString name)
{
  ROS_DEBUG_STREAM("State changed(event) to: " << name.toStdString() <<
                  "(" << value << ")");
  state_value_ = value;
  state_name_ = name;
  emit stateChanged(value, name);
}


bool StateMachine::setExecute(std::function<int ()> execute_method)
{
  ROS_INFO_STREAM("Initializing state machine with EXECUTE function pointer");
  return execute_->setOperationMethod(execute_method);
}

bool StateMachine::setResetting(std::function<int ()> resetting_method)
{
  ROS_INFO_STREAM("Initializing state machine with RESETTING function pointer");
  return resetting_->setOperationMethod(resetting_method);
}

void StateMachine::_start() {sm_internal_.postEvent(CmdEvent::start());}
void StateMachine::_clear() {sm_internal_.postEvent(CmdEvent::clear());}
void StateMachine::_reset() {sm_internal_.postEvent(CmdEvent::reset());}
void StateMachine::_hold() {sm_internal_.postEvent(CmdEvent::hold());}
void StateMachine::_unhold() {sm_internal_.postEvent(CmdEvent::unhold());}
void StateMachine::_suspend() {sm_internal_.postEvent(CmdEvent::suspend());}
void StateMachine::_unsuspend() {sm_internal_.postEvent(CmdEvent::unsuspend());}
void StateMachine::_stop() {sm_internal_.postEvent(CmdEvent::stop());}
void StateMachine::_abort() {sm_internal_.postEvent(CmdEvent::abort());}


ContinuousCycle::ContinuousCycle()
{

  ROS_INFO_STREAM("Forming CONTINUOUS CYCLE state machine (states + transitions)");


  //Naming <from state>_<to state>
  CmdTransition*                abortable_aborting_on_cmd = CmdTransition::abort(*abortable_, *aborting_);
  ErrorTransition*              abortable_aborting_on_error = new ErrorTransition(*abortable_, *aborting_);
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
  StateCompleteTransition*      execute_execute_ = new StateCompleteTransition(*execute_, *execute_);
  StateCompleteTransition*      completing_complete = new StateCompleteTransition(*completing_, *complete_);
  CmdTransition*                complete_resetting_ = CmdTransition::reset(*complete_, *resetting_);
  StateCompleteTransition*      resetting_idle_ = new StateCompleteTransition(*resetting_, *idle_);
  CmdTransition*                execute_suspending_ = CmdTransition::suspend(*execute_,*suspending_);
  StateCompleteTransition*      suspending_suspended_ = new StateCompleteTransition(*suspending_,*suspended_);
  CmdTransition*                suspended_unsuspending_ = CmdTransition::unsuspend(*suspended_, *unsuspending_);
  StateCompleteTransition*      unsuspending_execute_ = new StateCompleteTransition(*unsuspending_, *execute_);


  abortable_->setInitialState(clearing_);
  stoppable_->setInitialState(resetting_);
  sm_internal_.setInitialState(aborted_);
  ROS_INFO_STREAM("State machine formed");
}


SingleCycle::SingleCycle()
{

  ROS_INFO_STREAM("Forming SINGLE CYCLE state machine (states + transitions)");


  //Naming <from state>_<to state>
  CmdTransition*                abortable_aborting_on_cmd = CmdTransition::abort(*abortable_, *aborting_);
  ErrorTransition*              abortable_aborting_on_error = new ErrorTransition(*abortable_, *aborting_);
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


  abortable_->setInitialState(clearing_);
  stoppable_->setInitialState(resetting_);
  sm_internal_.setInitialState(aborted_);
  ROS_INFO_STREAM("State machine formed");
}



}//packml_sm
