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

#include "packml_sm/dlog.h"
#include "packml_sm/boost/packml_events.h"
#include "packml_sm/boost/packml_states.h"
#include "packml_sm/state_change_notifier.h"

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>

namespace packml_sm
{
struct PackmlTransitionsSingleCycle : public StateChangeNotifier,
                                      public boost::msm::front::state_machine_def<PackmlTransitionsSingleCycle>
{
public:
  typedef Aborted_impl initial_state;

  template <class FSM, class Event>
  void no_transition(Event const&, FSM&, int)
  {
  }

  struct transition_table
      : boost::mpl::vector<
            //    Start     Event        Target      Action                      Guard
            //   +---------+------------+-----------+---------------------------+----------------------------+
            // aborted
            boost::msm::front::Row<Aborted_impl, clear_event, Clearing_impl>,
            // clearing
            boost::msm::front::Row<Clearing_impl, state_complete_event, Stopped_impl>,
            boost::msm::front::Row<Clearing_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Clearing_impl, error_event, Aborting_impl>,
            // stopped
            boost::msm::front::Row<Stopped_impl, reset_event, Resetting_impl>,
            boost::msm::front::Row<Stopped_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Stopped_impl, error_event, Aborting_impl>,
            // resetting
            boost::msm::front::Row<Resetting_impl, state_complete_event, Idle_impl>,
            boost::msm::front::Row<Resetting_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Resetting_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Resetting_impl, stop_event, Stopping_impl>,
            // idle
            boost::msm::front::Row<Idle_impl, start_event, Starting_impl>,
            boost::msm::front::Row<Idle_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Idle_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Idle_impl, stop_event, Stopping_impl>,
            // starting
            boost::msm::front::Row<Starting_impl, state_complete_event, Execute_impl>,
            boost::msm::front::Row<Starting_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Starting_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Starting_impl, stop_event, Stopping_impl>,
            // execute
            boost::msm::front::Row<Execute_impl, hold_event, Holding_impl>,
            boost::msm::front::Row<Execute_impl, state_complete_event, Completing_impl>,
            boost::msm::front::Row<Execute_impl, suspend_event, Suspending_impl>,
            boost::msm::front::Row<Execute_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Execute_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Execute_impl, stop_event, Stopping_impl>,
            // holding
            boost::msm::front::Row<Holding_impl, state_complete_event, Held_impl>,
            boost::msm::front::Row<Holding_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Holding_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Holding_impl, stop_event, Stopping_impl>,
            // held
            boost::msm::front::Row<Held_impl, unhold_event, UnHolding_impl>,
            boost::msm::front::Row<Held_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Held_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Held_impl, stop_event, Stopping_impl>,
            // unholding
            boost::msm::front::Row<UnHolding_impl, state_complete_event, Execute_impl>,
            boost::msm::front::Row<UnHolding_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<UnHolding_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<UnHolding_impl, stop_event, Stopping_impl>,
            // suspending
            boost::msm::front::Row<Suspending_impl, state_complete_event, Suspended_impl>,
            boost::msm::front::Row<Suspending_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Suspending_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Suspending_impl, stop_event, Stopping_impl>,
            // suspended
            boost::msm::front::Row<Suspended_impl, unsuspend_event, UnSuspending_impl>,
            boost::msm::front::Row<Suspended_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Suspended_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Suspended_impl, stop_event, Stopping_impl>,
            // unsuspending
            boost::msm::front::Row<UnSuspending_impl, state_complete_event, Execute_impl>,
            boost::msm::front::Row<UnSuspending_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<UnSuspending_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<UnSuspending_impl, stop_event, Stopping_impl>,
            // completing
            boost::msm::front::Row<Completing_impl, state_complete_event, Complete_impl>,
            boost::msm::front::Row<Completing_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Completing_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Completing_impl, stop_event, Stopping_impl>,
            // complete
            boost::msm::front::Row<Complete_impl, reset_event, Resetting_impl>,
            boost::msm::front::Row<Complete_impl, abort_event, Aborting_impl>,
            boost::msm::front::Row<Complete_impl, error_event, Aborting_impl>,
            boost::msm::front::Row<Complete_impl, stop_event, Stopping_impl>,
            // aborting
            boost::msm::front::Row<Aborting_impl, state_complete_event, Aborted_impl>,
            // stopping
            boost::msm::front::Row<Stopping_impl, state_complete_event, Stopped_impl> >
  {
  };
};
}
