# This message contains machine agnostic information that is extracted from PackML
# standard states

std_msgs/Header header       # Time stamp when stats were captured
std_msgs/Duration duration      # Duration over which statistics are calculated

# State duration - Sum of state durations should equal total duraction above
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
float32 throughput              # Successful cycles/exe_duration above

# Derives/calculated statistics
# OEE - https://en.wikipedia.org/wiki/Overall_equipment_effectiveness
float32 availability # executing vs faulted
float32 performance # target rate vs cycle count
float32 quality # success vs fail
float32 overall_equipment_effectiveness # perf / quality
