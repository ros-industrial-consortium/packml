/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018 Plus One Robotics
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

#include "packml_sm/state_changed_event_args.h"

namespace packml_sm
{
struct StateChangeNotifier
{
public:
  EventHandler<StateChangeNotifier, StateChangedEventArgs> stateChangedEvent;

  void handleStateChangeNotify(const std::string& state_name, StatesEnum state_id)
  {
    stateChangedEvent.invoke(*this, StateChangedEventArgs(state_name, state_id));
  }
};
}
