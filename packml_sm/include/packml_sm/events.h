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

#ifndef EVENTS_H
#define EVENTS_H

#endif // EVENTS_H


#include "QEvent"
#include "QAbstractTransition"

#include "packml_sm/common.h"
#include "packml_sm/state.h"

namespace packml_sm
{


static int PACKML_CMD_EVENT_TYPE = QEvent::User+1;
static int PACKML_STATE_COMPLETE_EVENT_TYPE = QEvent::User+2;

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


struct StateCompleteEvent : public QEvent
{
  StateCompleteEvent()
    : QEvent(QEvent::Type(PACKML_STATE_COMPLETE_EVENT_TYPE)) {}
};


} //packml_sm
