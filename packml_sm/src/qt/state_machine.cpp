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
QCoreApplication* a;
void init(int argc, char* argv[])
{
  if (NULL == QCoreApplication::instance())
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

// NOTES:
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
  connect(abortable_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  stoppable_ = WaitState::Stoppable(abortable_);
  connect(stoppable_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  ROS_DEBUG_STREAM("Constructiong acting/wait states");
  unholding_ = ActingState::Unholding(stoppable_);
  connect(unholding_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  held_ = WaitState::Held(stoppable_);
  connect(held_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  holding_ = ActingState::Holding(stoppable_);
  connect(holding_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  idle_ = WaitState::Idle(stoppable_);
  connect(idle_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  starting_ = ActingState::Starting(stoppable_);
  connect(starting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  completing_ = ActingState::Completing(stoppable_);
  connect(completing_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  complete_ = WaitState::Complete(stoppable_);
  connect(complete_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  resetting_ = ActingState::Resetting(stoppable_);
  connect(resetting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  unsuspending_ = ActingState::Unsuspending(stoppable_);
  connect(unsuspending_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  suspended_ = WaitState::Suspended(stoppable_);
  connect(suspended_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  suspending_ = ActingState::Suspending(stoppable_);
  connect(suspending_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  stopped_ = WaitState::Stopped(abortable_);
  connect(stopped_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  stopping_ = ActingState::Stopping(abortable_);
  connect(stopping_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  clearing_ = ActingState::Clearing(abortable_);
  connect(clearing_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  aborted_ = WaitState::Aborted();
  connect(aborted_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  aborting_ = ActingState::Aborting();
  connect(aborting_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

  execute_ = ActingState::Execute(stoppable_);
  connect(execute_, SIGNAL(stateEntered(int, QString)), this, SLOT(setState(int, QString)));

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
  if (NULL == QCoreApplication::instance())
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
  ROS_DEBUG_STREAM("State changed(event) to: " << name.toStdString() << "(" << value << ")");
  state_value_ = value;
  state_name_ = name;
  emit stateChanged(value, name);
}

bool StateMachine::setStarting(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with STARTING function pointer");
  return starting_->setOperationMethod(state_method);
}

bool StateMachine::setCompleting(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with COMPLETING function pointer");
  return completing_->setOperationMethod(state_method);
}

bool StateMachine::setAborting(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with ABORTING function pointer");
  return aborting_->setOperationMethod(state_method);
}

bool StateMachine::setClearing(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with CLEARING function pointer");
  return clearing_->setOperationMethod(state_method);
}

bool StateMachine::setStopping(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with STOPPING function pointer");
  return stopping_->setOperationMethod(state_method);
}

bool StateMachine::setResetting(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with RESETTING function pointer");
  return resetting_->setOperationMethod(state_method);
}

bool StateMachine::setSuspending(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with SUSPENDING function pointer");
  return suspending_->setOperationMethod(state_method);
}

bool StateMachine::setUnsuspending(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with UNSUSPENDING function pointer");
  return unsuspending_->setOperationMethod(state_method);
}

bool StateMachine::setHolding(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with HOLDING function pointer");
  return holding_->setOperationMethod(state_method);
}

bool StateMachine::setUnholding(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with UNHOLDING function pointer");
  return unholding_->setOperationMethod(state_method);
}

bool StateMachine::setExecute(std::function<int()> state_method)
{
  ROS_INFO_STREAM("Initializing state machine with EXECUTE function pointer");
  return execute_->setOperationMethod(state_method);
}

void StateMachine::_start()
{
  sm_internal_.postEvent(CmdEvent::start());
}
void StateMachine::_clear()
{
  sm_internal_.postEvent(CmdEvent::clear());
}
void StateMachine::_reset()
{
  sm_internal_.postEvent(CmdEvent::reset());
}
void StateMachine::_hold()
{
  sm_internal_.postEvent(CmdEvent::hold());
}
void StateMachine::_unhold()
{
  sm_internal_.postEvent(CmdEvent::unhold());
}
void StateMachine::_suspend()
{
  sm_internal_.postEvent(CmdEvent::suspend());
}
void StateMachine::_unsuspend()
{
  sm_internal_.postEvent(CmdEvent::unsuspend());
}
void StateMachine::_stop()
{
  sm_internal_.postEvent(CmdEvent::stop());
}
void StateMachine::_abort()
{
  sm_internal_.postEvent(CmdEvent::abort());
}
}  // packml_sm
