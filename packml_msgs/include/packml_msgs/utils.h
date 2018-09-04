/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017 Shaun Edwards
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

#include <packml_msgs/Status.h>
#include "packml_sm/common.h"

namespace packml_msgs
{
bool isStandardState(int state)
{
  switch (state)
  {
    case State::ABORTED:
    case State::ABORTING:
    case State::CLEARING:
    case State::COMPLETE:
    case State::COMPLETING:
    case State::EXECUTE:
    case State::HELD:
    case State::HOLDING:
    case State::IDLE:
    case State::OFF:
    case State::RESETTING:
    case State::STARTING:
    case State::STOPPED:
    case State::STOPPING:
    case State::SUSPENDED:
    case State::SUSPENDING:
    case State::UNHOLDING:
    case State::UNSUSPENDING:
      return true;
      break;
    default:
      return false;
      break;
  }
}

Status initStatus(std::string node_name)
{
  Status status;
  status.header.frame_id = node_name;
  status.state.val = packml_msgs::State::UNDEFINED;
  status.sub_state = packml_msgs::State::UNDEFINED;
  status.mode.val = packml_msgs::Mode::UNDEFINED;
  // TODO: need better definition on errors
  status.error = 0;
  status.sub_error = 0;
  return status;
}

}  // namespace packml_msgs

#ifndef UTILS_H
#define UTILS_H

#endif  // UTILS_H
