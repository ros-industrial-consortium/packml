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

#include "packml_sm/transitions.h"
#include "packml_sm/events.h"
namespace packml_sm
{
ErrorTransition::ErrorTransition(PackmlState& from, PackmlState& to)
{
  this->setTargetState(&to);
  from.addTransition(this);
  ROS_INFO_STREAM("Creating error transition from " << from.name().toStdString() << " to " << to.name().toStdString());
}

bool ErrorTransition::eventTest(QEvent* e)
{
  if (e->type() != QEvent::Type(PACKML_ERROR_EVENT_TYPE))
    return false;
  return (true);
}
}