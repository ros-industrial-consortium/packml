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


#ifndef TRANSITIONS_H
#define TRANSITIONS_H


#include "QEvent"
#include "QAbstractTransition"

#include "packml_sm/common.h"
#include "packml_sm/state.h"

#include "ros/console.h"

namespace packml_sm
{




class CmdTransition : public QAbstractTransition
{
public:


  static CmdTransition* clear(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::CLEAR, "clear", from, to);
  }
  static CmdTransition* start(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::START, "start", from, to);
  }
  static CmdTransition* stop(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::STOP, "stop", from, to);
  }
  static CmdTransition* hold(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::HOLD, "hold", from, to);
  }
  static CmdTransition* abort(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::ABORT, "abort", from, to);
  }
  static CmdTransition* reset(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::RESET, "reset", from, to);
  }
  static CmdTransition* estop(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::ESTOP, "estop", from, to);
  }
  static CmdTransition* suspend(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::SUSPEND, "abort", from, to);
  }
  static CmdTransition* unsuspend(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::UNSUSPEND, "unsuspend", from, to);
  }
  static CmdTransition* unhold(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::UNHOLD, "unhold", from, to);
  }

  CmdTransition(const CmdEnum &cmd_value, const QString &name_value) :
    cmd(cmd_value),
    name(name_value) {}


  CmdTransition(const CmdEnum &cmd_value, const QString &name_value,
                PackmlState & from, PackmlState & to);

protected:


  virtual bool eventTest(QEvent *e);
  virtual void onTransition(QEvent *e) {}

  CmdEnum cmd;
  QString name;
};




class StateCompleteTransition : public QAbstractTransition
{
public:
  StateCompleteTransition() {}

  StateCompleteTransition(PackmlState & from, PackmlState & to);

  ~StateCompleteTransition() {}

protected:
  virtual bool eventTest(QEvent *e);
  virtual void onTransition(QEvent *e) {}

private:
};


class ErrorTransition : public QAbstractTransition
{
public:

  ErrorTransition() {}

  ErrorTransition(PackmlState & from, PackmlState & to);

  ~ErrorTransition() {}

protected:
  virtual bool eventTest(QEvent *e);
  virtual void onTransition(QEvent *e) {}


private:
};





}
#endif // TRANSITIONS_H
