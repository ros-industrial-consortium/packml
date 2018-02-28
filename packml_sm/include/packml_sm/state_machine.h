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


#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <functional>

#include <QtGui>
#include "QEvent"
#include "QAbstractTransition"

#include "packml_sm/state.h"
#include "packml_sm/transitions.h"

#include "ros/console.h"

namespace packml_sm
{

/**
 * @brief The StateMachineInterface class defines a implementation independent interface
 * to a PackML state machine.
 */
class StateMachineInterface
{
public:

  virtual bool activate()=0;
  virtual bool setStarting(std::function<int()> state_method)=0;
  virtual bool setExecute(std::function<int()> state_method)=0;
  virtual bool setCompleting(std::function<int()> state_method)=0;
  virtual bool setAborting(std::function<int()> state_method)=0;
  virtual bool setClearing(std::function<int()> state_method)=0;
  virtual bool setStopping(std::function<int()> state_method)=0;
  virtual bool setResetting(std::function<int()> state_method)=0;
  virtual bool setSuspending(std::function<int()> state_method)=0;
  virtual bool setUnsuspending(std::function<int()> state_method)=0;
  virtual bool setHolding(std::function<int()> state_method)=0;
  virtual bool setUnholding(std::function<int()> state_method)=0;
  virtual bool isActive()=0;
  virtual int getCurrentState()=0;

  virtual bool start();
  virtual bool clear();
  virtual bool reset();
  virtual bool hold();
  virtual bool unhold();
  virtual bool suspend();
  virtual bool unsuspend();
  virtual bool stop();
  virtual bool abort();

protected:

  virtual void _start()=0;
  virtual void _clear()=0;
  virtual void _reset()=0;
  virtual void _hold()=0;
  virtual void _unhold()=0;
  virtual void _suspend()=0;
  virtual void _unsuspend()=0;
  virtual void _stop()=0;
  virtual void _abort()=0;

};


void init(int argc, char *argv[]);


class StateMachine : public QObject, public StateMachineInterface
{
  Q_OBJECT

public:

  static std::shared_ptr<StateMachine> singleCyleSM();
  static std::shared_ptr<StateMachine> continuousCycleSM();

  bool activate();
  bool deactivate();
  bool setIdle(std::function<int()> state_method);
  bool setStarting(std::function<int()> state_method);
  bool setExecute(std::function<int()> state_method);
  bool setCompleting(std::function<int()> state_method);
  bool setAborting(std::function<int()> state_method);
  bool setClearing(std::function<int()> state_method);
  bool setStopping(std::function<int()> state_method);
  bool setResetting(std::function<int()> state_method);
  bool setSuspending(std::function<int()> state_method);
  bool setUnsuspending(std::function<int()> state_method);
  bool setHolding(std::function<int()> state_method);
  bool setUnholding(std::function<int()> state_method);

  bool isActive()
  {
    return sm_internal_.isRunning();
  }

  int getCurrentState()
  {
    return state_value_;
  }

  virtual ~StateMachine();


protected:
  StateMachine();
  virtual void _start();
  virtual void _clear();
  virtual void _reset();
  virtual void _hold();
  virtual void _unhold();
  virtual void _suspend();
  virtual void _unsuspend();
  virtual void _stop();
  virtual void _abort();

  int state_value_;
  QString state_name_;

  WaitState* abortable_;
  WaitState* stoppable_;
  WaitState* held_;
  WaitState* idle_;
  WaitState* suspended_;
  WaitState* stopped_;
  WaitState* complete_;
  WaitState* aborted_;

  ActingState* unholding_;
  ActingState* holding_;
  ActingState* starting_;
  ActingState* completing_;
  ActingState* resetting_;
  ActingState* unsuspending_;
  ActingState* suspending_;
  ActingState* stopping_;
  ActingState* clearing_;
  ActingState* aborting_;

  DualState* execute_;

  QStateMachine sm_internal_;



protected slots:
  void setState(int value, QString name);

signals:
  void stateChanged(int value, QString name);

};

class ContinuousCycle : public StateMachine
{
  Q_OBJECT

public:

  ContinuousCycle();
  virtual ~ContinuousCycle() {}

};

class SingleCycle : public StateMachine
{
  Q_OBJECT

public:

  SingleCycle();
  virtual ~SingleCycle() {}

};

}

#endif //STATE_MACHINE_H
