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

#include "packml_sm/qt/continuous_cycle.h"
#include "packml_sm/dlog.h"

namespace packml_sm
{
ContinuousCycle::ContinuousCycle()
{
  DLog::LogInfo("Forming CONTINUOUS CYCLE state machine (states + transitions)");

  // Naming <from state>_<to state>
  CmdTransition* abortable_aborting_on_cmd = CmdTransition::abort(*abortable_, *aborting_);
  ErrorTransition* abortable_aborting_on_error = new ErrorTransition(*abortable_, *aborting_);
  StateCompleteTransition* aborting_aborted = new StateCompleteTransition(*aborting_, *aborted_);
  CmdTransition* aborted_clearing_ = CmdTransition::clear(*aborted_, *clearing_);
  StateCompleteTransition* clearing_stopped_ = new StateCompleteTransition(*clearing_, *stopped_);
  CmdTransition* stoppable_stopping_ = CmdTransition::stop(*stoppable_, *stopping_);
  StateCompleteTransition* stopping_stopped = new StateCompleteTransition(*stopping_, *stopped_);
  CmdTransition* stopped_resetting_ = CmdTransition::reset(*stopped_, *resetting_);
  StateCompleteTransition* unholding_execute_ = new StateCompleteTransition(*unholding_, *execute_);
  CmdTransition* held_unholding_ = CmdTransition::unhold(*held_, *unholding_);
  StateCompleteTransition* holding_held_ = new StateCompleteTransition(*holding_, *held_);
  CmdTransition* idle_starting_ = CmdTransition::start(*idle_, *starting_);
  StateCompleteTransition* starting_execute_ = new StateCompleteTransition(*starting_, *execute_);
  CmdTransition* execute_holding_ = CmdTransition::hold(*execute_, *holding_);
  StateCompleteTransition* execute_execute_ = new StateCompleteTransition(*execute_, *execute_);
  StateCompleteTransition* completing_complete = new StateCompleteTransition(*completing_, *complete_);
  CmdTransition* complete_resetting_ = CmdTransition::reset(*complete_, *resetting_);
  StateCompleteTransition* resetting_idle_ = new StateCompleteTransition(*resetting_, *idle_);
  CmdTransition* execute_suspending_ = CmdTransition::suspend(*execute_, *suspending_);
  StateCompleteTransition* suspending_suspended_ = new StateCompleteTransition(*suspending_, *suspended_);
  CmdTransition* suspended_unsuspending_ = CmdTransition::unsuspend(*suspended_, *unsuspending_);
  StateCompleteTransition* unsuspending_execute_ = new StateCompleteTransition(*unsuspending_, *execute_);

  abortable_->setInitialState(clearing_);
  stoppable_->setInitialState(resetting_);
  sm_internal_.setInitialState(aborted_);
  DLog::LogInfo("State machine formed");
}
}
