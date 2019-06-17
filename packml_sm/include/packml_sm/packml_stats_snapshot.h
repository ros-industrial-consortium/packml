#ifndef PACKML_STATS_SNAPSHOT_H
#define PACKML_STATS_SNAPSHOT_H

#include <map>
#include "packml_sm/packml_stats_itemized.h"

namespace packml_sm
{
/**
 * @brief Container for the current packml statistics snap shot.
 *
 */
struct PackmlStatsSnapshot
{
  double duration;                       /** duration over which the stats were captured */
  double idle_duration;                  /** time spent in the idle state */
  double exe_duration;                   /** time spent in the execute state */
  double held_duration;                  /** time spent in the held state */
  double susp_duration;                  /** time spent in the suspended state */
  double cmplt_duration;                 /** time spent in the complete state */
  double stop_duration;                  /** time spent in the stopped state */
  double abort_duration;                 /** time spent in the aborted state */
  int cycle_count;                       /** total cycle count */
  int success_count;                     /** total success count */
  int fail_count;                        /** total fail count */
  float throughput;                      /** the computed throughput */
  float availability;                    /** the computed availability */
  float performance;                     /** the computed performance */
  float quality;                         /** the computed quality */
  float overall_equipment_effectiveness; /** the computed overal equipment effectiveness */
  std::map<int16_t, PackmlStatsItemized> itemized_error_map;
  std::map<int16_t, PackmlStatsItemized> itemized_quality_map;
};
}

#endif  // PACKML_STATS_SNAPSHOT_H
