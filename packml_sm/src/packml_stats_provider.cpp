#include "packml_sm/packml_stats_provider.h"

#include <limits>

namespace packml_sm
{
PackmlStatsProvider::PackmlStatsProvider(std::shared_ptr<AbstractStateMachine> state_machine)
  : state_machine_(state_machine)
{
}

void PackmlStatsProvider::start()
{
  start_time_ = std::chrono::steady_clock::now();
}

void PackmlStatsProvider::reset()
{
  state_machine_->resetStats();

  if (state_machine_->isActive())
  {
    start();
  }
}

void PackmlStatsProvider::incrementCycleCount(bool success)
{
  if (success)
  {
    success_count_++;
  }
  else
  {
    failure_count_++;
  }
}

void PackmlStatsProvider::setTargetRate(float target_rate)
{
  target_rate_ = target_rate;
}

double PackmlStatsProvider::totalDuration() const
{
  auto end_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> delta = end_time - start_time_;

  return delta.count();
}

double PackmlStatsProvider::idleDuration() const
{
  return state_machine_->getIdleTime();
}

double PackmlStatsProvider::executeDuration() const
{
  return state_machine_->getExecuteTime();
}

double PackmlStatsProvider::heldDuration() const
{
  return state_machine_->getHeldTime();
}

double PackmlStatsProvider::suspendedDuration() const
{
  return state_machine_->getSuspendedTime();
}

double PackmlStatsProvider::completedDuration() const
{
  return state_machine_->getCompleteTime();
}

double PackmlStatsProvider::stoppedDuration() const
{
  return state_machine_->getStoppedTime();
}

double PackmlStatsProvider::abortedDuration() const
{
  return state_machine_->getAbortedTime();
}

int PackmlStatsProvider::cycleCount() const
{
  return success_count_ + failure_count_;
}

int PackmlStatsProvider::successCount() const
{
  return success_count_;
}

int PackmlStatsProvider::failureCount() const
{
  return failure_count_;
}

float PackmlStatsProvider::throughput() const
{
  auto duration = totalDuration();
  if (duration > std::numeric_limits<double>::epsilon())
  {
    return static_cast<float>(cycleCount()) / static_cast<float>(duration);
  }

  return 0.0f;
}

float PackmlStatsProvider::availabilty() const
{
  // executing vs faulted
  return 0.0f;
}

float PackmlStatsProvider::performance() const
{
  if (cycleCount() != 0)
  {
    return target_rate_ / static_cast<float>(cycleCount());
  }

  return 0.0f;
}

float PackmlStatsProvider::quality() const
{
  if (failure_count_ != 0)
  {
    return static_cast<float>(success_count_) / static_cast<float>(failure_count_);
  }

  return 0.0f;
}

float PackmlStatsProvider::overallEquipmentEffectiveness() const
{
  if (quality() > std::numeric_limits<float>::epsilon())
  {
    return performance() / quality();
  }

  return 0.0f;
}
}
