#ifndef PACKML_STATS_ITEMIZED_H
#define PACKML_STATS_ITEMIZED_H

#include <cstdint>

namespace packml_sm
{
/**
 * @brief Container for the current packml statistics snap shot.
 *
 */
struct PackmlStatsItemized
{
  int16_t id;
  int32_t count;
  double duration;
};
}  // namespace packml_sm

#endif  // PACKML_STATS_ITEMIZED_H