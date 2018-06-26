#pragma once

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/mpl/vector.hpp>

// packml states
struct Aborted_impl : public boost::msm::front::state<>
{
};

struct Clearing_impl : public boost::msm::front::state<>
{
};

struct Stopped_impl : public boost::msm::front::state<>
{
};

struct Resetting_impl : public boost::msm::front::state<>
{
};

struct Idle_impl : public boost::msm::front::state<>
{
};

struct Starting_impl : public boost::msm::front::state<>
{
};

struct Execute_impl : public boost::msm::front::state<>
{
};

struct Holding_impl : public boost::msm::front::state<>
{
};

struct Held_impl : public boost::msm::front::state<>
{
};

struct UnHolding_impl : public boost::msm::front::state<>
{
};

struct Suspending_impl : public boost::msm::front::state<>
{
};

struct Suspended_impl : public boost::msm::front::state<>
{
};

struct UnSuspending_impl : public boost::msm::front::state<>
{
};

struct Completing_impl : public boost::msm::front::state<>
{
};

struct Complete_impl : public boost::msm::front::state<>
{
};

struct Aborting_impl : public boost::msm::front::state<>
{
};

struct Stopping_impl : public boost::msm::front::state<>
{
};

// packml events
struct clear_impl
{
};

struct state_complete_impl
{
};

struct abort_impl
{
};

struct reset_impl
{
};

struct stop_impl
{
};

struct start_impl
{
};

struct hold_impl
{
};

struct suspend_impl
{
};

struct unhold_impl
{
};

struct unsuspend_impl
{
};

struct packml_transition_table
    : boost::mpl::vector<
          //    Start     Event        Target      Action                      Guard
          //   +---------+------------+-----------+---------------------------+----------------------------+
          // aborted
          boost::msm::front::Row<Aborted_impl(), clear_impl(), Clearing_impl()>,
          // clearing
          boost::msm::front::Row<Clearing_impl(), state_complete_impl(), Stopped_impl()>,
          boost::msm::front::Row<Clearing_impl(), abort_impl(), Aborting_impl()>,
          // stopped
          boost::msm::front::Row<Stopped_impl(), reset_impl(), Resetting_impl()>,
          boost::msm::front::Row<Stopped_impl(), abort_impl(), Aborting_impl()>,
          // resetting
          boost::msm::front::Row<Resetting_impl(), state_complete_impl(), Idle_impl()>,
          boost::msm::front::Row<Resetting_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Resetting_impl(), stop_impl(), Stopping_impl()>,
          // idle
          boost::msm::front::Row<Idle_impl(), start_impl(), Starting_impl()>,
          boost::msm::front::Row<Idle_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Idle_impl(), stop_impl(), Stopping_impl()>,
          // starting
          boost::msm::front::Row<Starting_impl(), state_complete_impl(), Execute_impl()>,
          boost::msm::front::Row<Starting_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Starting_impl(), stop_impl(), Stopping_impl()>,
          // execute
          boost::msm::front::Row<Execute_impl(), hold_impl(), Holding_impl()>,
          boost::msm::front::Row<Execute_impl(), state_complete_impl(), Completing_impl()>,
          boost::msm::front::Row<Execute_impl(), suspend_impl(), Suspending_impl()>,
          boost::msm::front::Row<Execute_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Execute_impl(), stop_impl(), Stopping_impl()>,
          // holding
          boost::msm::front::Row<Holding_impl(), state_complete_impl(), Held_impl()>,
          boost::msm::front::Row<Holding_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Holding_impl(), stop_impl(), Stopping_impl()>,
          // held
          boost::msm::front::Row<Held_impl(), unhold_impl(), UnHolding_impl()>,
          boost::msm::front::Row<Held_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Held_impl(), stop_impl(), Stopping_impl()>,
          // unholding
          boost::msm::front::Row<UnHolding_impl(), state_complete_impl(), Execute_impl()>,
          boost::msm::front::Row<UnHolding_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<UnHolding_impl(), stop_impl(), Stopping_impl()>,
          // suspending
          boost::msm::front::Row<Suspending_impl(), state_complete_impl(), Suspended_impl()>,
          boost::msm::front::Row<Suspending_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Suspending_impl(), stop_impl(), Stopping_impl()>,
          // suspended
          boost::msm::front::Row<Suspended_impl(), unsuspend_impl(), UnSuspending_impl()>,
          boost::msm::front::Row<Suspended_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Suspended_impl(), stop_impl(), Stopping_impl()>,
          // unsuspending
          boost::msm::front::Row<UnSuspending_impl(), state_complete_impl(), Execute_impl()>,
          boost::msm::front::Row<UnSuspending_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<UnSuspending_impl(), stop_impl(), Stopping_impl()>,
          // completing
          boost::msm::front::Row<Completing_impl(), state_complete_impl(), Complete_impl()>,
          boost::msm::front::Row<Completing_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Completing_impl(), stop_impl(), Stopping_impl()>,
          // complete
          boost::msm::front::Row<Complete_impl(), reset_impl(), Resetting_impl()>,
          boost::msm::front::Row<Complete_impl(), abort_impl(), Aborting_impl()>,
          boost::msm::front::Row<Complete_impl(), stop_impl(), Stopping_impl()>,
          // aborting
          boost::msm::front::Row<Aborting_impl(), state_complete_impl(), Aborted_impl()>,
          // stopping
          boost::msm::front::Row<Stopping_impl(), state_complete_impl(), Stopped_impl()> >
{
};
