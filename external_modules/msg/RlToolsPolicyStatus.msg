# TOPICS rl_tools_policy_status
uint64 timestamp
uint64 timestamp_sample
bool subscription_update_angular_velocity
bool subscription_update_local_position
bool subscription_update_visual_odometry
bool subscription_update_attitude
bool subscription_update_rl_tools_command

uint8 exit_reason 
uint8 EXIT_REASON_NONE                               = 0
uint8 EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE         = 1
uint8 EXIT_REASON_NOT_ALL_OBSERVATIONS_SET           = 2
uint8 EXIT_REASON_ANGULAR_VELOCITY_STALE             = 3
uint8 EXIT_REASON_LOCAL_POSITION_STALE               = 4
uint8 EXIT_REASON_VISUAL_ODOMETRY_STALE              = 5
uint8 EXIT_REASON_VISUAL_ODOMETRY_WRONG_FRAME        = 6
uint8 EXIT_REASON_VISUAL_ODOMETRY_NAN                = 7
uint8 EXIT_REASON_ATTITUDE_STALE                     = 8
uint8 EXIT_REASON_EXECUTOR_STATUS_SOURCE_NOT_CONTROL = 9

uint8 ODOMETRY_SOURCE_LOCAL_POSITION = 0
uint8 ODOMETRY_SOURCE_VISUAL_ODOMETRY = 1
uint8 ODOMETRY_SOURCE_UNKNOWN = 2

bool command_stale

bool active
uint8 substep
uint8 odometry_source
float32 control_interval

uint32 visual_odometry_stale_counter
uint64 visual_odometry_age
float32 visual_odometry_dt_mean
float32 visual_odometry_dt_std
uint8 NUM_VISUAL_ODOMETRY_DT_MAX = 3
float32[3] visual_odometry_dt_max