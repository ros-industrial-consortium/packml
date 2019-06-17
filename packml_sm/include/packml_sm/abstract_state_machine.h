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
#include "packml_sm/packml_stats_snapshot.h"
#include "packml_sm/packml_stats_itemized.h"
#include "common.h"

#include <map>
#include <mutex>
#include <chrono>
#include <functional>

namespace packml_sm
{
/**
 * @brief The StateMachineInterface class defines a implementation independent interface
 * to a PackML state machine.
 */
class AbstractStateMachine
{
public:
  EventHandler<AbstractStateMachine, StateChangedEventArgs> stateChangedEvent; /** Triggered during a state changed
                                                                                  event. */

  /**
   * @brief Constructor for AbstractStateMachine.
   *
   */
  AbstractStateMachine();

  /**
   * @brief Destructor for AbstractStateMachine.
   *
   */
  virtual ~AbstractStateMachine()
  {
  }

  /**
   * @brief Override to handle activate command.
   *
   * @return bool Returns true on success.
   */
  virtual bool activate() = 0;

  /**
   * @brief Override to handle deactivate command.
   *
   * @return bool Returns true on success.
   */
  virtual bool deactivate() = 0;

  /**
   * @brief  Override to handle setting the starting state method.
   *
   * @param state_method The state method to execute when entering the start state.
   * @return bool Returns true on success.
   */
  virtual bool setStarting(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the execute state method.
   *
   * @param state_method The state method to execute when entering the exec state.
   * @return bool Returns true on success.
   */
  virtual bool setExecute(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the completing state method.
   *
   * @param state_method The state method to execute when entering the completing state.
   * @return bool Returns true on success.
   */
  virtual bool setCompleting(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the aborting state method.
   *
   * @param state_method The state method to execute when entering the aborting state.
   * @return bool Returns true on success.
   */
  virtual bool setAborting(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the clearing state method.
   *
   * @param state_method The state method to execute when entering the clearing state.
   * @return bool Returns true on success.
   */
  virtual bool setClearing(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the stopping state method.
   *
   * @param state_method The state method to execute when entering the stopping state.
   * @return bool Returns true on success.
   */
  virtual bool setStopping(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the resetting state method.
   *
   * @param state_method The state method to execute when entering the resetting state.
   * @return bool Returns true on success.
   */
  virtual bool setResetting(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the suspending state method.
   *
   * @param state_method The state method to execute when entering the suspending state.
   * @return bool Returns true on success.
   */
  virtual bool setSuspending(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the un-suspending state method.
   *
   * @param state_method The state method to execute when entering the unsuspending state.
   * @return bool Returns true on success.
   */
  virtual bool setUnsuspending(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting the holding state method.
   *
   * @param state_method The state method to execute when entering the holding state.
   * @return bool Returns true on success.
   */
  virtual bool setHolding(std::function<int()> state_method) = 0;

  /**
   * @brief Override to handle setting unholding state method.
   *
   * @param state_method The state method to execute when entering the unholding state.
   * @return bool Returns true on success.
   */
  virtual bool setUnholding(std::function<int()> state_method) = 0;

  /**
   * @brief Override to provide an accessor to whether the state machine is currently active.
   *
   * @return bool Returns true if active.
   */
  virtual bool isActive() = 0;

  /**
   * @brief Accessor for the current state.
   *
   * @return StatesEnum The current state.
   */
  StatesEnum getCurrentState() const
  {
    return current_state_;
  }

  /**
   * @brief Accessor for the duration spent in idle time.
   *
   * @return double Returns the time spent in the idle state.
   */
  double getIdleTime();

  /**
   * @brief Accessor for the duration spent in starting.
   *
   * @return double Returns the time spent in the starting state.
   */
  double getStartingTime();

  /**
   * @brief Accessor for the duration spent in resetting.
   *
   * @return double Returns the time spent in the resetting state.
   */
  double getResettingTime();

  /**
   * @brief Accessor for the duration spent in execute.
   *
   * @return double Returns the time spent in the execute state.
   */
  double getExecuteTime();

  /**
   * @brief Accessor for the duration spent in held.
   *
   * @return double Returns the time spent in the held state.
   */
  double getHeldTime();

  /**
   * @brief Accessor for the duration spent in holding.
   *
   * @return double Returns the time spent in the holding state.
   */
  double getHoldingTime();

  /**
   * @brief Accessor for the duration spent in unholding.
   *
   * @return double Returns the time spent in the unholding state.
   */
  double getUnholdingTime();

  /**
   * @brief Accessor for the duration spent in suspended.
   *
   * @return double Returns the time spent in the suspended state.
   */
  double getSuspendedTime();

  /**
   * @brief Accessor for the duration spent in suspending.
   *
   * @return double Returns the time spent in the suspending state.
   */
  double getSuspendingTime();

  /**
   * @brief Accessor for the duration spent in unsuspending.
   *
   * @return double Returns the time spent in the unsuspending state.
   */
  double getUnsuspendingTime();

  /**
   * @brief Accessor for the duration spent in complete.
   *
   * @return double Returns the time spent in the complete state.
   */
  double getCompleteTime();

  /**
   * @brief Accessor for the duration spent in stopped.
   *
   * @return double Returns the time spent in the stopped state.
   */
  double getStoppedTime();

  /**
   * @brief Accessor for the duration spent in clearing.
   *
   * @return double Returns the time spent in the clearing state.
   */
  double getClearingTime();

  /**
   * @brief Accessor for the duration spent in stopping.
   *
   * @return double Returns the time spent in the stopping state.
   */
  double getStoppingTime();

  /**
   * @brief Accessor for the duration spent in aborted.
   *
   * @return double Returns the time spent in the aborted state.
   */
  double getAbortedTime();

  /**
   * @brief Accessor for the duration spent in aborting.
   *
   * @return double Returns the time spent in the aborting state.
   */
  double getAbortingTime();

  /**
   * @brief Reset all of the tracked states.
   *
   */
  void resetStats();

  /**
   * @brief Call to increment or add a specific Itemized error stat.
   *
   */
  void incrementErrorStatItem(int16_t id, int32_t count, double duration);

  /**
   * @brief Call to increment or add a specific Itemized quality stat.
   *
   */
  void incrementQualityStatItem(int16_t id, int32_t count, double duration);

  /**
   * @brief Call to increment the successful operation count.
   *
   */
  void incrementSuccessCount();

  /**
   * @brief Call to increment the failed operation count.
   *
   */
  void incrementFailureCount();

  /**
   * @brief Sets the ideal cycle time in operations per second.
   *
   * @param ideal_cycle_time The ideal cycle time in operations per second.
   */
  void setIdealCycleTime(float ideal_cycle_time);

  /**
   * @brief Call to send the start command.
   *
   * @return bool Returns true on success.
   */
  virtual bool start();

  /**
   * @brief Call to send the clear command.
   *
   * @return bool Returns true on success.
   */
  virtual bool clear();

  /**
   * @brief Call to send the reset command.
   *
   * @return bool Returns true on success.
   */
  virtual bool reset();

  /**
   * @brief Call to send the hold command.
   *
   * @return bool Returns true on success.
   */
  virtual bool hold();

  /**
   * @brief Call to send the unhold command.
   *
   * @return bool Returns true on success.
   */
  virtual bool unhold();

  /**
   * @brief Call to send the suspend command.
   *
   * @return bool Returns true on success.
   */
  virtual bool suspend();

  /**
   * @brief Call to send the unsuspend command.
   *
   * @return bool Returns true on success.
   */
  virtual bool unsuspend();

  /**
   * @brief Call to send the stop command.
   *
   * @return bool Returns true on success.
   */
  virtual bool stop();

  /**
   * @brief Call to send the abort command.
   *
   * @return bool Returns true on success.
   */
  virtual bool abort();

  /**
   * @brief Fills the reference variable with the current stats snapshot.
   *
   * @param snapshot_out Reference to the variable to fill the snapshot data with.
   */
  void getCurrentStatSnapshot(PackmlStatsSnapshot& snapshot_out);

protected:
  /**
   * @brief Call to invoke a state changed event.
   *
   * @param name The name of the new state.
   * @param value The name of the new value.
   */
  void invokeStateChangedEvent(const std::string& name, StatesEnum value);

  /**
   * @brief Override to call implementations version of start command.
   *
   */
  virtual void _start() = 0;

  /**
   * @brief Override to call implementations version of clear command.
   *
   */
  virtual void _clear() = 0;

  /**
   * @brief Override to call implementations version of the reset command.
   *
   */
  virtual void _reset() = 0;

  /**
   * @brief Override to call implementations version of the hold command.
   *
   */
  virtual void _hold() = 0;

  /**
   * @brief Override to call implementations version of the unhold command.
   *
   */
  virtual void _unhold() = 0;

  /**
   * @brief Override to call implementations version of the suspend command.
   *
   */
  virtual void _suspend() = 0;

  /**
   * @brief Override to call implementations version of the unsuspend command.
   *
   */
  virtual void _unsuspend() = 0;

  /**
   * @brief Override to call implementations version of the stop command.
   *
   */
  virtual void _stop() = 0;

  /**
   * @brief Override to call implementations version of the abort command.
   *
   */
  virtual void _abort() = 0;

private:
  std::map<int16_t, PackmlStatsItemized> itemized_error_map_;
  std::map<int16_t, PackmlStatsItemized> itemized_quality_map_;
  int success_count_ = 0;        /** number of successful operations */
  int failure_count_ = 0;        /** number of failed operations */
  float ideal_cycle_time_ = 0.0; /** ideal cycle time in operations per second */

  std::recursive_mutex stat_mutex_;                  /** stat mutex for protecting stat operations */
  StatesEnum current_state_ = StatesEnum::UNDEFINED; /** cache of the current state */
  std::map<StatesEnum, double> duration_map_; /** container for all of the durations referenced by their state id */
  std::chrono::steady_clock::time_point start_time_; /** start time for the latest state entry */

  /**
   * @brief adds or updates the specific itemized map
   *
   * @param itemized_map the map to add or update.
   * @param id the id to add or update.
   * @param count the number of occurences to add.
   * @param duration the duration to add.
   */
  void incrementMapStatItem(std::map<int16_t, PackmlStatsItemized>& itemized_map, int16_t id, int32_t count,
                            double duration);

  /**
   * @brief Updates all of the durations for the states based on the new state.
   *
   * @param new_state The new state entry.
   */
  void updateClock(StatesEnum new_state);

  /**
   * @brief Accessor for the given state duration.
   *
   * @param state The state of interest.
   * @return double Returns the total time spent in the given state.
   */
  double getStateDuration(StatesEnum state);

  /**
   * @brief Accessor for the total duration of the state machine.
   *
   * @return double Returns the total time spent in the state machine.
   */
  double calculateTotalTime();
};
}
