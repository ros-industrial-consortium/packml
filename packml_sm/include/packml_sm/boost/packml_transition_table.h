#pragma once

#include "packml_sm/boost/packml_events.h"
#include "packml_sm/boost/packml_states.h"

#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/back/state_machine.hpp>

namespace packml_sm
{
struct packml_transition_table
    : boost::mpl::vector<
          //    Start     Event        Target      Action                      Guard
          //   +---------+------------+-----------+---------------------------+----------------------------+
          // aborted
          boost::msm::front::Row<Aborted_impl(), clear_event(), Clearing_impl()>,
          // clearing
          boost::msm::front::Row<Clearing_impl(), state_complete_event(), Stopped_impl()>,
          boost::msm::front::Row<Clearing_impl(), abort_event(), Aborting_impl()>,
          // stopped
          boost::msm::front::Row<Stopped_impl(), reset_event(), Resetting_impl()>,
          boost::msm::front::Row<Stopped_impl(), abort_event(), Aborting_impl()>,
          // resetting
          boost::msm::front::Row<Resetting_impl(), state_complete_event(), Idle_impl()>,
          boost::msm::front::Row<Resetting_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Resetting_impl(), stop_event(), Stopping_impl()>,
          // idle
          boost::msm::front::Row<Idle_impl(), start_event(), Starting_impl()>,
          boost::msm::front::Row<Idle_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Idle_impl(), stop_event(), Stopping_impl()>,
          // starting
          boost::msm::front::Row<Starting_impl(), state_complete_event(), Execute_impl()>,
          boost::msm::front::Row<Starting_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Starting_impl(), stop_event(), Stopping_impl()>,
          // execute
          boost::msm::front::Row<Execute_impl(), hold_event(), Holding_impl()>,
          boost::msm::front::Row<Execute_impl(), state_complete_event(), Completing_impl()>,
          boost::msm::front::Row<Execute_impl(), suspend_event(), Suspending_impl()>,
          boost::msm::front::Row<Execute_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Execute_impl(), stop_event(), Stopping_impl()>,
          // holding
          boost::msm::front::Row<Holding_impl(), state_complete_event(), Held_impl()>,
          boost::msm::front::Row<Holding_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Holding_impl(), stop_event(), Stopping_impl()>,
          // held
          boost::msm::front::Row<Held_impl(), unhold_event(), UnHolding_impl()>,
          boost::msm::front::Row<Held_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Held_impl(), stop_event(), Stopping_impl()>,
          // unholding
          boost::msm::front::Row<UnHolding_impl(), state_complete_event(), Execute_impl()>,
          boost::msm::front::Row<UnHolding_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<UnHolding_impl(), stop_event(), Stopping_impl()>,
          // suspending
          boost::msm::front::Row<Suspending_impl(), state_complete_event(), Suspended_impl()>,
          boost::msm::front::Row<Suspending_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Suspending_impl(), stop_event(), Stopping_impl()>,
          // suspended
          boost::msm::front::Row<Suspended_impl(), unsuspend_event(), UnSuspending_impl()>,
          boost::msm::front::Row<Suspended_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Suspended_impl(), stop_event(), Stopping_impl()>,
          // unsuspending
          boost::msm::front::Row<UnSuspending_impl(), state_complete_event(), Execute_impl()>,
          boost::msm::front::Row<UnSuspending_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<UnSuspending_impl(), stop_event(), Stopping_impl()>,
          // completing
          boost::msm::front::Row<Completing_impl(), state_complete_event(), Complete_impl()>,
          boost::msm::front::Row<Completing_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Completing_impl(), stop_event(), Stopping_impl()>,
          // complete
          boost::msm::front::Row<Complete_impl(), reset_event(), Resetting_impl()>,
          boost::msm::front::Row<Complete_impl(), abort_event(), Aborting_impl()>,
          boost::msm::front::Row<Complete_impl(), stop_event(), Stopping_impl()>,
          // aborting
          boost::msm::front::Row<Aborting_impl(), state_complete_event(), Aborted_impl()>,
          // stopping
          boost::msm::front::Row<Stopping_impl(), state_complete_event(), Stopped_impl()> >
{
};
}
