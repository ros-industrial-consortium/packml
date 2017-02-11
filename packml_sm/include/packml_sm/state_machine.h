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

  virtual bool init(std::function<void()> execute_method)=0;
  virtual bool isActive()=0;
  virtual int getCurrentState()=0;

};


class StateMachine : public QStateMachine, StateMachineInterface
{
  Q_OBJECT

public:
  StateMachine();
  bool init(PackmlState* execute_state);
  bool init(std::function<void()> execute_method);

  bool isActive()
  {
    return isRunning();
  }

  int getCurrentState()
  {
    return state_value_;
  }

  virtual ~StateMachine() {}


protected:

  int state_value_;
  QString state_name_;

protected slots:
  void setState(int value, QString name);

};

}

#endif //STATE_MACHINE_H
