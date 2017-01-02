# This message contains machine agnostic information that is extracted from PackML
# standard states

std_msgs/Header stamp       # Duration of statistics calculation
std_msgs/Duration duration      # Duration over which statistics are calculated

# State duration - total duration should equal duration
std_msgs/Duration idle_duration
std_msgs/Duration exe_duration
std_msgs/Duration held_duration
std_msgs/Duration susp_duration
std_msgs/Duration cmplt_duration
std_msgs/Duration stop_duration
std_msgs/Duration abort_duration

ItemizedStats[] error_items          # Each item represents a machine fault
ItemizedStats[] quality_items        # Each item represents a quality state

int32 cycle_count               # General cycle count
int32 success_count             # Number of successful cycles
int32 fail_count                # Number of failure cycles
float32 throughput              # Successful cycles/run-time

# Derives/calculated statistics
# OEE - https://en.wikipedia.org/wiki/Overall_equipment_effectiveness
float32 availability
float32 performance
float32 quality
float32 oee
