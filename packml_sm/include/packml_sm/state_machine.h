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

#include <QtGui>
#include "QEvent"
#include "QAbstractTransition"

#include "packml_sm/state.h"
#include "packml_sm/transitions.h"

#include "ros/console.h"

namespace packml_sm
{


enum class StatesEnum;  //foward declaration of enum
class StateMachine;



class StateMachine : public QStateMachine
{
  Q_OBJECT

public:
  StateMachine();
  StateMachine(PackmlState* execute_state);
  void init(PackmlState* execute_state);
  int getCurrentState()
  {
    return state_value_;
  }

  virtual ~StateMachine() {}


protected:
  QThread worker_;
  int state_value_;
  QString state_name_;

protected slots:
  void setState(int value, QString name);

};

}

#endif //STATE_MACHINE_H
