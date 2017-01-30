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

#pragma once

#include <QtGui>
#include "QEvent"
#include "QAbstractTransition"

#include "packml_msgs/State.h"
#include "packml_msgs/StateCommand.h"
#include "ros/console.h"

namespace packml_sm
{

enum class StatesEnum;  //foward declaration of enum
struct BaseState : public QState
{
public:
  BaseState(StatesEnum state_value, QString name_value) :
    state_(state_value),
    name_(name_value) {}

  BaseState(StatesEnum state_value, QString name_value, QState* super_state) :
    QState(super_state),
    state_(state_value),
    name_(name_value) {}

  StatesEnum state() const {return state_;}
  const QString name() const {return name_;}

protected:
  StatesEnum state_;
  QString name_;
};


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
  COMPLETE = 106,

  // Super states that encapsulate multiple substates with a common transition
  // Not explicitly used in the standard but helpful for consutrcting the state
  // machine.
  ABORTABLE = 200,
  STOPPABLE = 201
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
  static CmdEvent* clear()
  {
    return new CmdEvent(CmdEnum::CLEAR);
  }
  static CmdEvent* start()
  {
    return new CmdEvent(CmdEnum::START);
  }
  static CmdEvent* stop()
  {
    return new CmdEvent(CmdEnum::STOP);
  }
  static CmdEvent* hold()
  {
    return new CmdEvent(CmdEnum::HOLD);
  }
  static CmdEvent* abort()
  {
    return new CmdEvent(CmdEnum::ABORT);
  }
  static CmdEvent* reset()
  {
    return new CmdEvent(CmdEnum::RESET);
  }
  static CmdEvent* estop()
  {
    return new CmdEvent(CmdEnum::ESTOP);
  }
  static CmdEvent* suspend()
  {
    return new CmdEvent(CmdEnum::SUSPEND);
  }
  static CmdEvent* unsuspend()
  {
    return new CmdEvent(CmdEnum::UNSUSPEND);
  }
  static CmdEvent* unhold()
  {
    return new CmdEvent(CmdEnum::UNHOLD);
  }

  CmdEvent(const CmdEnum &cmd_value)
    : QEvent(QEvent::Type(PACKML_CMD_EVENT_TYPE)),
      cmd(cmd_value) {}

  CmdEnum cmd;
};

class CmdTransition : public QAbstractTransition
{
public:


  static CmdTransition* clear(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::CLEAR, "clear", from, to);
  }
  static CmdTransition* start(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::START, "start", from, to);
  }
  static CmdTransition* stop(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::STOP, "stop", from, to);
  }
  static CmdTransition* hold(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::HOLD, "hold", from, to);
  }
  static CmdTransition* abort(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::ABORT, "abort", from, to);
  }
  static CmdTransition* reset(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::RESET, "reset", from, to);
  }
  static CmdTransition* estop(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::ESTOP, "estop", from, to);
  }
  static CmdTransition* suspend(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::SUSPEND, "abort", from, to);
  }
  static CmdTransition* unsuspend(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::UNSUSPEND, "unsuspend", from, to);
  }
  static CmdTransition* unhold(BaseState & from, BaseState & to)
  {
    return new CmdTransition(CmdEnum::UNHOLD, "unhold", from, to);
  }

  CmdTransition(const CmdEnum &cmd_value, const QString &name_value) :
    cmd(cmd_value),
    name(name_value) {}


  CmdTransition(const CmdEnum &cmd_value, const QString &name_value,
                BaseState & from, BaseState & to) :
    cmd(cmd_value),
    name(name_value)
  {
    this->setTargetState(&to);
    from.addTransition(this);
    ROS_INFO_STREAM("Creating " << this->name.toStdString() << " transition from " <<
                    from.name().toStdString() << " to " << to.name().toStdString());
  }

protected:


  virtual bool eventTest(QEvent *e)
  {
    ROS_INFO_STREAM("Testing event type: " << e->type());
    if (e->type() != QEvent::Type(PACKML_CMD_EVENT_TYPE))
      return false;
    CmdEvent *se = static_cast<CmdEvent*>(e);
    ROS_INFO_STREAM("Type cmd: " << cmd << ", event cmd: " << se->cmd);
    return (cmd == se->cmd);
  }

  virtual void onTransition(QEvent *e) {}

  CmdEnum cmd;
  QString name;
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

  StateCompleteTransition(BaseState & from, BaseState & to)
  {
    this->setTargetState(&to);
    from.addTransition(this);
    ROS_INFO_STREAM("Creating state complete transition from " <<
                    from.name().toStdString() << " to " << to.name().toStdString());
  }

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

struct WaitState : public BaseState
{
public:

  static WaitState* Abortable()
  {
    return new WaitState(StatesEnum::ABORTABLE, CmdEnum::ABORT, "Abortable");
  }
  static WaitState* Stoppable(QState* abortable)
  {
    return new WaitState(StatesEnum::STOPPABLE, CmdEnum::ABORT, "Stoppable", abortable);
  }
  static WaitState* Idle(QState* stoppable)
  {
    return new WaitState(StatesEnum::IDLE, CmdEnum::START, "Idle", stoppable);
  }
  static WaitState* Held(QState* stoppable)
  {
    return new WaitState(StatesEnum::HELD, CmdEnum::UNHOLD, "Held", stoppable);
  }
  static WaitState* Complete(QState* stoppable)
  {
    return new WaitState(StatesEnum::COMPLETE, CmdEnum::RESET, "Complete", stoppable);
  }
  static WaitState* Suspended(QState* stoppable)
  {
    return new WaitState(StatesEnum::SUSPENDED, CmdEnum::UNSUSPEND, "Suspended", stoppable);
  }
  static WaitState* Stopped(QState* abortable)
  {
    return new WaitState(StatesEnum::STOPPED, CmdEnum::RESET, "Stopped", abortable);
  }
  static WaitState* Aborted()
  {
    return new WaitState(StatesEnum::ABORTED, CmdEnum::CLEAR, "Aborted");
  }

  WaitState(StatesEnum state_value, CmdEnum exit_cmd_value, QString name_value) :
    BaseState(state_value, name_value),
    exit_cmd(exit_cmd_value)
  {}

  WaitState(StatesEnum state_value, CmdEnum exit_cmd_value, QString name_value, QState* super_state) :
    BaseState(state_value, name_value, super_state),
    exit_cmd(exit_cmd_value)
  {}
protected:
  virtual void onEntry(QEvent *e)
  {
    ROS_INFO_STREAM("Entering state: " << name_.toStdString() << "(" << state_ <<")");
  }
  virtual void onExit(QEvent *e)
  {
    ROS_INFO_STREAM("Exiting state: " << name_.toStdString() << "(" << state_ << ")");
  }

private:
  CmdEnum exit_cmd;
};


struct ActingState : public BaseState
{
public:

  static ActingState* Resetting(QState* stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::RESETTING, "Resetting", stoppable, delay_ms_value);
  }
  static ActingState* Starting(QState* stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::STARTING, "Starting", stoppable, delay_ms_value);
  }
  static ActingState* Unholding(QState* stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::UNHOLDING, "Un-Holding", stoppable, delay_ms_value);
  }
  static ActingState* Unsuspending(QState* stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::UNSUSPENDING, "Un-Suspending", stoppable, delay_ms_value);
  }
  static ActingState* Holding(QState* stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::HOLDING, "Holding", stoppable, delay_ms_value);
  }
  static ActingState* Suspending(QState* stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::SUSPENDING, "Suspending", stoppable, delay_ms_value);
  }
  static ActingState* Execute(QState* stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::EXECUTE, "Exectue", stoppable, delay_ms_value);
  }
  static ActingState* Completing(QState* stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::COMPLETING, "Completing", stoppable, delay_ms_value);
  }
  static ActingState* Aborting(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::ABORTING, "Aborting", delay_ms_value);
  }
  static ActingState* Clearing(QState* abortable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::CLEARING, "Clearing", abortable, delay_ms_value);
  }
  static ActingState* Stopping(QState* abortable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::STOPPING, "Aborting", abortable, delay_ms_value);
  }

  ActingState(StatesEnum state_value, const char* name_value, int delay_ms_value = 200) :
    BaseState(state_value, QString(name_value)),
    delay_ms(delay_ms_value)
  {}

  ActingState(StatesEnum state_value, const QString & name_value, QState* super_state, int delay_ms_value = 200) :
    BaseState(state_value, name_value, super_state),
    delay_ms(delay_ms_value)
  {}
protected:
  virtual void onEntry(QEvent *e)
  {

    ROS_INFO_STREAM("Entering state: " << name_.toStdString() << "(" << state_ <<")");
    StateCompleteEvent* sc = new StateCompleteEvent();
    machine()->postDelayedEvent(sc, delay_ms);  //sc is deletecd by state machine
  }

  virtual void onExit(QEvent *e)
  {
    ROS_INFO_STREAM("Leaving state: " << name_.toStdString() << "(" << state_ <<")");
  }
private:
  int delay_ms;
};

class StateMachine : public QStateMachine
{
  Q_OBJECT

public:
  StateMachine();
  StateMachine(BaseState* execute_state);
  void init(BaseState* execute_state);
  virtual ~StateMachine();


protected:
  QThread worker_;

};

}
