# Collector -> leader, published periodically.
uint8 RESULT_NONE=0
uint8 RESULT_COLLECTED=1
uint8 RESULT_FAILED=2

string robot                  # the collector's namespace, e.g. bz2
string state                  # STARTUP, IDLE, GOING, SEEKING, TARGETING, ... DETACHING
uint32 task_id                # task in progress, 0 when idle
uint32 last_task_id           # most recently finished task, 0 if none yet
uint8 last_result             # RESULT_* for last_task_id
string last_detail            # why it failed, for the leader's log
bool localised                # map->base_link available
float64 x                     # pose in the shared map frame
float64 y
float64 yaw
bool home_set
float64 home_x                # where this collector delivers cubes
float64 home_y
uint32 delivered
bool last_released            # last_task_id ended with the cube released short of HOME
float64 release_x             # where that cube was left (map frame), if last_released
float64 release_y
