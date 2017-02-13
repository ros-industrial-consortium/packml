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

#ifndef PACKML_STATE_H
#define PACKML_STATE_H

#include <functional>

#include <QtGui>
#include "QState"
#include "QEvent"
#include "QAbstractTransition"

#include "ros/console.h"
#include "packml_sm/common.h"

namespace packml_sm
{

struct PackmlState : public QState
{
  Q_OBJECT

public:
  PackmlState(StatesEnum state_value, QString name_value) :
    state_(state_value),
    name_(name_value),
    cummulative_time_(0) {}

  PackmlState(StatesEnum state_value, QString name_value, QState* super_state) :
    QState(super_state),
    state_(state_value),
    name_(name_value),
    cummulative_time_(0) {}

  StatesEnum state() const {return state_;}
  const QString name() const {return name_;}

signals:
  void stateEntered(int value, QString name);

protected:
  StatesEnum state_;
  QString name_;

  ros::Time enter_time_;
  ros::Time exit_time_;
  ros::Duration cummulative_time_;

  virtual void onEntry(QEvent *e);
  virtual void operation() {}
  virtual void onExit(QEvent *e);
};



struct WaitState : public PackmlState
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
    PackmlState(state_value, name_value),
    exit_cmd(exit_cmd_value)
  {}

  WaitState(StatesEnum state_value, CmdEnum exit_cmd_value, QString name_value, QState* super_state) :
    PackmlState(state_value, name_value, super_state),
    exit_cmd(exit_cmd_value)
  {}

private:
  CmdEnum exit_cmd;
};


struct ActingState : public PackmlState
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
    return new ActingState(StatesEnum::EXECUTE, "Execute", stoppable, delay_ms_value);
  }
  static ActingState* Execute(std::function<void()> function_value)
  {
    return new ActingState(StatesEnum::EXECUTE, "Execute", function_value);
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
    PackmlState(state_value, QString(name_value)),
    delay_ms(delay_ms_value)
  {}

  ActingState(StatesEnum state_value, const QString & name_value, QState* super_state, int delay_ms_value = 200) :
    PackmlState(state_value, name_value, super_state),
    delay_ms(delay_ms_value)
  {}

  ActingState(StatesEnum state_value, const char* name_value, std::function<void()> function_value) :
    PackmlState(state_value, QString(name_value)),
    function_(function_value)
  {}

  bool setOperationMethod(std::function<void()> function_value)
  {
    function_ = function_value;
    return true;
  }

protected:

  virtual void operation();

private:
  int delay_ms;
  std::function<void()> function_;
};

struct FunctionalState : public PackmlState
{
public:



protected:

  virtual void operation();

private:



};


}

#endif // PACKML_STATE_H
