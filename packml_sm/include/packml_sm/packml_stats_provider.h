#ifndef PACKML_STATS_PROVIDER_H
#define PACKML_STATS_PROVIDER_H

#include "packml_sm/abstract_state_machine.h"

#include <memory>
#include <chrono>

namespace packml_sm
{
class PackmlStatsProvider
{
public:
  explicit PackmlStatsProvider(std::shared_ptr<AbstractStateMachine> state_machine);

  void start();
  void reset();

  void incrementCycleCount(bool success);
  void setTargetRate(float target_rate);

  double totalDuration() const;
  double idleDuration() const;
  double executeDuration() const;
  double heldDuration() const;
  double suspendedDuration() const;
  double completedDuration() const;
  double stoppedDuration() const;
  double abortedDuration() const;

  int cycleCount() const;
  int successCount() const;
  int failureCount() const;

  float throughput() const;

  float availabilty() const;
  float performance() const;
  float quality() const;
  float overallEquipmentEffectiveness() const;

private:
  std::shared_ptr<AbstractStateMachine> state_machine_;
  std::chrono::steady_clock::time_point start_time_;
  int success_count_ = 0;
  int failure_count_ = 0;
  float target_rate_;
};
}

#endif  // PACKML_STATS_PROVIDER_H
