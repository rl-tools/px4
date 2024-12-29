# TOPICS rl_tools_policy_input
uint64 timestamp
uint64 timestamp_sample
bool active
uint8 STATE_OBSERVATION_DIM = 18
float32[18] state_observation
uint8 ACTION_HISTORY_LENGTH = 16
uint8 ACTION_DIM = 4
float32[128] action_history
