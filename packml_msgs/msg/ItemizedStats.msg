# This message contains itemized statiistics for PackML statistics reporting
# Each item containds a unique ID that defines the "item".  
# Example usage includes, error and warning events.

int16 id                          # unique identifier (user defined)
int32 count                       # number of occurances
std_msgs/Duration duration        # duration in "item" state
