# TOPICS rl_tools_policy_status
uint64 timestamp
uint64 timestamp_sample
uint8 subscription_update
uint8 SUBSCRIPTION_UPDATE_ANGULAR_VELOCITY = 1
uint8 SUBSCRIPTION_UPDATE_LOCAL_POSITION   = 2
uint8 SUBSCRIPTION_UPDATE_ATTITUDE         = 4
uint8 SUBSCRIPTION_UPDATE_RL_TOOLS_COMMAND = 8

uint8 exit_reason 
uint8 EXIT_REASON_NONE                       = 0
uint8 EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE = 1
uint8 EXIT_REASON_NOT_ALL_OBSERVATIONS_SET   = 2
uint8 EXIT_REASON_ANGULAR_VELOCITY_STALE     = 3
uint8 EXIT_REASON_LOCAL_POSITION_STALE       = 4
uint8 EXIT_REASON_ATTITUDE_STALE             = 5

bool command_stale
bool active

uint8 substep
float32 control_interval

