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
#include "packml_msgs/State.h"
#include "packml_msgs/StateCommand.h"
#include "ros/console.h"

namespace packml_sm
{

//This magic function allows iostream (i.e. ROS_##_STREAM) macros to print out
//enumerations
//see: http://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

static int PACKML_CMD_EVENT_TYPE = QEvent::User+1;
static int PACKML_STATE_COMPLETE_EVENT_TYPE = QEvent::User+2;

enum class StatesEnum {
  UNDEFINED = 0,
  OFF = 1,
  STOPPED = 2,
  STARTING = 3,
  IDLE = 4,
  SUSPENDED = 5,
  EXECUTE = 6,
  STOPPING = 7,
  ABORTING = 8,
  ABORTED = 9,
  HOLDING = 10,
  HELD = 11,

  RESETTING = 100,
  SUSPENDING = 101,
  UNSUSPENDING = 102,
  CLEARING = 103,
  UNHOLDING = 104,
  COMPLETING = 105,
  COMPLETE = 106
};

enum class ModeEnum {
  UNDEFINED = 0,
  AUTOMATIC = 1,
  SEMI_AUTOMATIC = 2,
  MANUAL = 3,
  IDLE = 4,
  SETUP = 11
};

enum class CmdEnum {
  UNDEFINED = 0,
  CLEAR = 1,
  START = 2,
  STOP = 3,
  HOLD = 4,
  ABORT = 5,
  RESET = 6,
  ESTOP = 7,

  SUSPEND = 100,
  UNSUSPEND = 101,
  UNHOLD = 102
};

struct CmdEvent : public QEvent
{
    CmdEvent(const int &cmd_value)
    : QEvent(QEvent::Type(PACKML_CMD_EVENT_TYPE)),
      cmd(cmd_value) {}

    int cmd;
};



class CmdTransition : public QAbstractTransition
{
public:
    CmdTransition(const int &cmd_value)
        : cmd(cmd_value) {}

protected:
    virtual bool eventTest(QEvent *e)
    {
        if (e->type() != QEvent::Type(PACKML_CMD_EVENT_TYPE))
            return false;
        CmdEvent *se = static_cast<CmdEvent*>(e);
        return (cmd == se->cmd);
    }

    virtual void onTransition(QEvent *e) {}

private:
    int cmd;
};


struct StateCompleteEvent : public QEvent
{
    StateCompleteEvent()
    : QEvent(QEvent::Type(PACKML_STATE_COMPLETE_EVENT_TYPE)) {}
};


class StateCompleteTransition : public QAbstractTransition
{
public:
    StateCompleteTransition() {}
    ~StateCompleteTransition() {}

protected:
    virtual bool eventTest(QEvent *e)
    {
      if (e->type() != QEvent::Type(PACKML_STATE_COMPLETE_EVENT_TYPE))
          return false;
      return (true);
    }

    virtual void onTransition(QEvent *e) {}

private:
};

struct BaseState : public QState
{
public:
  BaseState(StatesEnum state_value, QString name_value) :
    state(state_value),
    name(name_value) {}

protected:
  StatesEnum state;
  QString name;
};

struct WaitState : public BaseState
{
public:
  WaitState(StatesEnum state_value, int exit_cmd_value, QString name_value) :
    BaseState(state_value, name_value),
    exit_cmd(exit_cmd_value)
  {}
protected:
  virtual void onEntry(QEvent *e){}
  virtual void onExit(QEvent *e){}
private:
  int exit_cmd;
};


struct ActingState : public BaseState
{
public:

  static ActingState* Resetting(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::RESETTING, "Resetting", delay_ms_value);
  }
  static ActingState* Starting(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::STARTING, "Starting", delay_ms_value);
  }
  static ActingState* Unholding(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::UNHOLDING, "Un-Holding", delay_ms_value);
  }
  static ActingState* Unsuspending(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::UNSUSPENDING, "Un-Suspending", delay_ms_value);
  }
  static ActingState* Holding(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::HOLDING, "Holding", delay_ms_value);
  }
  static ActingState* Suspending(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::SUSPENDING, "Suspending", delay_ms_value);
  }
  static ActingState* Completing(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::COMPLETING, "Completing", delay_ms_value);
  }
  static ActingState* Aborting(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::ABORTING, "Aborting", delay_ms_value);
  }
  static ActingState* Clearing(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::CLEARING, "Clearing", delay_ms_value);
  }
  static ActingState* Stopping(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::STOPPING, "Aborting", delay_ms_value);
  }

  ActingState(StatesEnum state_value, const char* name_value, int delay_ms_value = 200) :
    BaseState(state_value, QString(name_value)),
    delay_ms(delay_ms_value)
  {}

  ActingState(StatesEnum state_value, const QString & name_value, int delay_ms_value = 200) :
    BaseState(state_value, name_value),
    delay_ms(delay_ms_value)
  {}
protected:
  virtual void onEntry(QEvent *e)
  {

    ROS_INFO_STREAM("Entering state: " << name.toStdString() << "(" << state <<")");
    StateCompleteEvent* sc = new StateCompleteEvent();
    machine()->postDelayedEvent(sc, delay_ms);  //sc is deletecd by state machine
  }

  virtual void onExit(QEvent *e)
  {
    ROS_INFO_STREAM("Leaving state: " << name.toStdString() << "(" << state <<")");
  }
private:
  int delay_ms;
};

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
  ROS_INFO_STREAM("State machine constructor");

//  QState* idle = new WaitState();
//  QState* held = new WaitState();
  ROS_INFO_STREAM("Forming state machine (states + transitions)");
  ROS_INFO_STREAM("Constructiong states");
  QState* stopping = ActingState::Stopping();
  QState* holding = ActingState::Holding();

  ROS_INFO_STREAM("Construction and loading transitions");

  StateCompleteTransition* stop_trans = new StateCompleteTransition();
  stop_trans->setTargetState(holding);
  stopping->addTransition(stop_trans);

  StateCompleteTransition* hold_trans = new StateCompleteTransition();
  hold_trans->setTargetState(stopping);
  holding->addTransition(hold_trans);


  ROS_INFO_STREAM("Adding states to state machine");
  addState(stopping);
  addState(holding);
  setInitialState(stopping);
  ROS_INFO_STREAM("State machine formed");
  this->

  moveToThread(&worker_);
  worker_.start();

}
StateMachine::~StateMachine()
{
}




}
