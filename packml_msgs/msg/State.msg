# PackML State, based on the PackML standard, summarized here:
# http://www.plcopen.org/pages/promotion/publications/downloads/mapping_omac_statediagram.pdf

# TODO: Find out if PackML defines state values - I can't find anything thus far.

# Acting (transition) states
# Acting state mnemonic state XYZIJK:
# - XYZ - From state (0 indicates entire group)
# - IJK - To state
int32 RESETTING = 120111
int32 STARTING = 111112
int32 HOLDING = 112113
int32 UNHOLDING = 113112
int32 SUSPENDING = 112114
int32 UNSUSPENDING = 114112
int32 COMPLETING = 112115
int32 STOPPING = 110120
int32 CLEARING = 211120
int32 ABORTING = 100211

# Wait state
# Wait state mnemonic state ABC:
# - A: Major grouping
# - B: Minor grouping
# - C: State
int32 IDLE = 111
int32 EXECUTE = 112
int32 HELD = 113
int32 SUSPENDED = 114
int32 COMPLETE = 115
int32 STOPPED = 120
int32 ABORTED = 211
int32 val
