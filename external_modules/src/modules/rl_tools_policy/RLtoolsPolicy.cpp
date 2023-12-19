#include "RLtoolsPolicy.hpp"

RLtoolsPolicy::RLtoolsPolicy(): ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl){
	rlt::malloc(device, buffers);
	rlt::malloc(device, input);
	rlt::malloc(device, output);

	for(TI step_i = 0; step_i < ACTION_HISTORY_LENGTH; step_i++){
		for(TI action_i = 0; action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM; action_i++){
			action_history[step_i][action_i] = 0;
		}
	}
	// node state
	timestamp_last_angular_velocity_set = false;
	timestamp_last_local_position_set = false;
	timestamp_last_attitude_set = false;
	timestamp_last_command_set = false;
	previous_command_stale = false;
	timeout_message_sent = false;

	// controller state
	// timestamp_last_forward_pass_set = false;
	controller_tick = 0;

	_actuator_motors_rl_tools_pub.advertise();
}

RLtoolsPolicy::~RLtoolsPolicy()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	rlt::free(device, buffers);
	rlt::free(device, input);
	rlt::free(device, output);
}

bool RLtoolsPolicy::init()
{
	this->init_time = hrt_absolute_time();
	// ScheduleOnInterval(500_us); // 2000 us interval, 200 Hz rate
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position_sub callback registration failed");
		return false;
	}
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity_sub callback registration failed");
		return false;
	}
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude_sub callback registration failed");
		return false;
	}

	auto input_sample = rlt::row(device, rl_tools::checkpoint::observation::container, 0);
	auto output_sample = rlt::row(device, output, 0);
	uint32_t start, end;
	start = hrt_absolute_time();
	rlt::evaluate(device, rl_tools::checkpoint::actor::model, input_sample, output_sample, buffers);
	end = hrt_absolute_time();
	T inference_frequency = (T)1.0/((T)(end - start)/1e6);
	PX4_INFO("rl_tools_benchmark: %dHz", (int)(inference_frequency));
	
	T abs_error = 0;
	constexpr T EPSILON = 1e-6;

	for(TI batch_i = 0; batch_i < BATCH_SIZE; batch_i++){
		for(TI output_i = 0; output_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM; output_i++){
			T abs_diff = rlt::get(output, batch_i, output_i) - rlt::get(rl_tools::checkpoint::action::container, batch_i, output_i);
			PX4_INFO("output[%d][%d]: %f (diff %f)", batch_i, output_i, get(output, batch_i, output_i), abs_diff);
		}
	}

	return abs_error < EPSILON;
}
template <typename T>
T clip(T x, T max, T min){
	if(x > max){
		return max;
	}
	if(x < min){
		return min;
	}
	return x;
}
template <typename OBS_SPEC>
void RLtoolsPolicy::observe_rotation_matrix(rlt::Matrix<OBS_SPEC>& observation, TestObservationMode mode){
	using T = typename OBS_SPEC::T;
	// converting from FRD to FLU
    static_assert(OBS_SPEC::ROWS == 1);
    static_assert(OBS_SPEC::COLS == 18);
	T qw = 1, qx = 0, qy = 0, qz = 0;
	if(mode >= TestObservationMode::ORIENTATION){
		qw = +_vehicle_attitude.q[0];
		qx = +_vehicle_attitude.q[1];
		qy = -_vehicle_attitude.q[2];
		qz = -_vehicle_attitude.q[3];
	}
	if(mode >= TestObservationMode::POSITION){
		T x = +(_vehicle_local_position.x - _rl_tools_command.target_position[0]);
		T y = -(_vehicle_local_position.y - _rl_tools_command.target_position[1]);
		T z = -(_vehicle_local_position.z - _rl_tools_command.target_position[2]);
		rlt::set(observation, 0,  0 + 0, clip(x, MAX_POSITION_ERROR, -MAX_POSITION_ERROR));
		rlt::set(observation, 0,  0 + 1, clip(y, MAX_POSITION_ERROR, -MAX_POSITION_ERROR));
		rlt::set(observation, 0,  0 + 2, clip(z, MAX_POSITION_ERROR, -MAX_POSITION_ERROR));
	}
	else{
		rlt::set(observation, 0,  0 + 0, 0);
		rlt::set(observation, 0,  0 + 1, 0);
		rlt::set(observation, 0,  0 + 2, 0);
	}
    rlt::set(observation, 0,  3 + 0, (1 - 2*qy*qy - 2*qz*qz));
    rlt::set(observation, 0,  3 + 1, (    2*qx*qy - 2*qw*qz));
    rlt::set(observation, 0,  3 + 2, (    2*qx*qz + 2*qw*qy));
    rlt::set(observation, 0,  3 + 3, (    2*qx*qy + 2*qw*qz));
    rlt::set(observation, 0,  3 + 4, (1 - 2*qx*qx - 2*qz*qz));
    rlt::set(observation, 0,  3 + 5, (    2*qy*qz - 2*qw*qx));
    rlt::set(observation, 0,  3 + 6, (    2*qx*qz - 2*qw*qy));
    rlt::set(observation, 0,  3 + 7, (    2*qy*qz + 2*qw*qx));
    rlt::set(observation, 0,  3 + 8, (1 - 2*qx*qx - 2*qy*qy));
	if(mode >= TestObservationMode::LINEAR_VELOCITY){
		T vx = +_vehicle_local_position.vx;
		T vy = -_vehicle_local_position.vy;
		T vz = -_vehicle_local_position.vz;
		rlt::set(observation, 0, 12 + 0, clip(vx, MAX_VELOCITY_ERROR, -MAX_VELOCITY_ERROR));
		rlt::set(observation, 0, 12 + 1, clip(vy, MAX_VELOCITY_ERROR, -MAX_VELOCITY_ERROR));
		rlt::set(observation, 0, 12 + 2, clip(vz, MAX_VELOCITY_ERROR, -MAX_VELOCITY_ERROR));
	}
	else{
		rlt::set(observation, 0, 12 + 0, 0);
		rlt::set(observation, 0, 12 + 1, 0);
		rlt::set(observation, 0, 12 + 2, 0);
	}
	if(mode >= TestObservationMode::ANGULAR_VELOCITY){
		rlt::set(observation, 0, 15 + 0, +_vehicle_angular_velocity.xyz[0]);
		rlt::set(observation, 0, 15 + 1, -_vehicle_angular_velocity.xyz[1]);
		rlt::set(observation, 0, 15 + 2, -_vehicle_angular_velocity.xyz[2]);
	}
	else{
		rlt::set(observation, 0, 15 + 0, 0);
		rlt::set(observation, 0, 15 + 1, 0);
		rlt::set(observation, 0, 15 + 2, 0);

	}
}
void RLtoolsPolicy::rl_tools_control(TI substep, TestObservationMode mode){
    auto state_rotation_matrix_input = rlt::view(device, input, rlt::matrix::ViewSpec<1, 18>{}, 0, 0);
    observe_rotation_matrix(state_rotation_matrix_input, mode);
    auto action_history_observation = rlt::view(device, input, rlt::matrix::ViewSpec<1, ACTION_HISTORY_LENGTH * rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM>{}, 0, 18);
    for(TI step_i = 0; step_i < ACTION_HISTORY_LENGTH; step_i++){
        for(TI action_i = 0; action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM; action_i++){
            rlt::set(action_history_observation, 0, step_i * rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM + action_i, action_history[step_i][action_i]);
        }
    }
    rlt::evaluate(device, rl_tools::checkpoint::actor::model, input, output, buffers);
	// for(TI action_i = 0; action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM; action_i++){
	// 	set(output, 0, action_i, 0.1);
	// }
    if(substep == 0){
        for(TI step_i = 0; step_i < ACTION_HISTORY_LENGTH - 1; step_i++){
            for(TI action_i = 0; action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM; action_i++){
                action_history[step_i][action_i] = action_history[step_i + 1][action_i];
            }
        }
    }
    for(TI action_i = 0; action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM; action_i++){
        T value = action_history[ACTION_HISTORY_LENGTH - 1][action_i];
        value *= substep;
        value += rlt::get(output, 0, action_i);
        value /= substep + 1;
        action_history[ACTION_HISTORY_LENGTH - 1][action_i] = value;
    }
}

void RLtoolsPolicy::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		_vehicle_attitude_sub.unregisterCallback();
		_vehicle_local_position_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	perf_count(_loop_interval_perf);

	perf_begin(_loop_perf);
	uint32_t current_time = hrt_absolute_time();

	rl_tools_policy_status_s status;
	status.timestamp = current_time;
	status.timestamp_sample = current_time;
	status.subscription_update = 0x0;
	status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_NONE;
	status.substep = 0;

	bool angular_velocity_update = false;
	if(_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)){
		timestamp_last_angular_velocity = current_time;
		timestamp_last_angular_velocity_set = true;
		angular_velocity_update = true;
		status.subscription_update |= rl_tools_policy_status_s::SUBSCRIPTION_UPDATE_ANGULAR_VELOCITY;
	}
	if(_vehicle_local_position_sub.update(&_vehicle_local_position)){
		timestamp_last_local_position = current_time;
		timestamp_last_local_position_set = true;
		status.subscription_update |= rl_tools_policy_status_s::SUBSCRIPTION_UPDATE_LOCAL_POSITION;
	}
	if(_vehicle_attitude_sub.update(&_vehicle_attitude)){
		timestamp_last_attitude = current_time;
		timestamp_last_attitude_set = true;
		status.subscription_update |= rl_tools_policy_status_s::SUBSCRIPTION_UPDATE_ATTITUDE;
	}
	if(_rl_tools_command_sub.update(&_rl_tools_command)){
		timestamp_last_command = current_time;
		timestamp_last_command_set = true;
		previous_command_stale = false;
		status.subscription_update |= rl_tools_policy_status_s::SUBSCRIPTION_UPDATE_RL_TOOLS_COMMAND;
	}

	// if(_manual_control_input_sub.update(&_manual_control_input)) {
	// 	timestamp_last_manual_control_input_set = true;
	// 	timestamp_last_manual_control_input = current_time;
	// }

	if(!angular_velocity_update){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE;
		_rl_tools_policy_status_pub.publish(status);
		return;
	}

	if(!timestamp_last_angular_velocity_set || !timestamp_last_local_position_set || !timestamp_last_attitude_set){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_NOT_ALL_OBSERVATIONS_SET;
		_rl_tools_policy_status_pub.publish(status);
		return;
	}

	if((current_time - timestamp_last_angular_velocity) > OBSERVATION_TIMEOUT_ANGULAR_VELOCITY){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_ANGULAR_VELOCITY_STALE;
		_rl_tools_policy_status_pub.publish(status);
		if(!timeout_message_sent){
			PX4_ERR("angular velocity timeout");
			timeout_message_sent = true;
		}
		return;
	}
	if((current_time - timestamp_last_local_position) > OBSERVATION_TIMEOUT_POSITION){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_LOCAL_POSITION_STALE;
		_rl_tools_policy_status_pub.publish(status);
		if(!timeout_message_sent){
			PX4_ERR("local position timeout");
			timeout_message_sent = true;
		}
		return;
	}
	if((current_time - timestamp_last_attitude) > OBSERVATION_TIMEOUT_ATTITUDE){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_ATTITUDE_STALE;
		_rl_tools_policy_status_pub.publish(status);
		if(!timeout_message_sent){
			PX4_ERR("attitude timeout");
			timeout_message_sent = true;
		}
		return;
	}
	if(!timestamp_last_command_set || (current_time - timestamp_last_command) > COMMAND_TIMEOUT){
		status.command_stale = true;
		if(!previous_command_stale){
			PX4_WARN("Command turned stale at: %f %f %f", _vehicle_local_position.x, _vehicle_local_position.y, _vehicle_local_position.z);
			_rl_tools_command.target_position[0] = _vehicle_local_position.x;
			_rl_tools_command.target_position[1] = _vehicle_local_position.y;
			_rl_tools_command.target_position[2] = _vehicle_local_position.z;
		}
		previous_command_stale = true;
	}
	else{
		status.command_stale = false;
	}
	// if(SCALE_OUTPUT_WITH_THROTTLE && ((current_time - timestamp_last_manual_control_input) > MANUAL_CONTROL_TIMEOUT)){
	// 	if(!timeout_message_sent){
	// 		PX4_ERR("manual control input timeout");
	// 		timeout_message_sent = true;
	// 	}
	// 	return;
	// }

	timeout_message_sent = false;

	// if(timestamp_last_forward_pass_set){
	// 	if((current_time - timestamp_last_forward_pass) < CONTROL_INTERVAL){
	// 		return;
	// 	}
	// }

    TI substep = controller_tick % CONTROL_MULTIPLE;
	status.substep = substep;
	rl_tools_control(substep, TEST_OBSERVATION_MODE);

	for(TI state_i = 0; state_i < 18; state_i++){
		status.state_observation[state_i] = rlt::get(input, 0, state_i);
	}
	_rl_tools_policy_status_pub.publish(status);

	controller_tick++;
	// timestamp_last_forward_pass = current_time;
	// timestamp_last_forward_pass_set = true;

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = hrt_absolute_time();
	for(TI action_i=0; action_i < actuator_motors_s::NUM_CONTROLS; action_i++){
		if(action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM){
			T value = rlt::get(output, 0, action_i);
			value = (value + 1) / 2;
			// T scaled_value = value * (SCALE_OUTPUT_WITH_THROTTLE ? (_manual_control_input.throttle + 1)/2 : 0.5);
			constexpr T training_min = 0.1;
			constexpr T training_max = 0.5;
			T scaled_value = (training_max - training_min) * value;
			actuator_motors.control[action_i] = scaled_value;
		}
		else{
			actuator_motors.control[action_i] = NAN;
		}
	}
	_actuator_motors_rl_tools_pub.publish(actuator_motors);
	perf_end(_loop_perf);
}

int RLtoolsPolicy::task_spawn(int argc, char *argv[])
{
	RLtoolsPolicy *instance = new RLtoolsPolicy();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RLtoolsPolicy::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int RLtoolsPolicy::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RLtoolsPolicy::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLtools Policy

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rl_tools_benchmark", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rl_tools_policy_main(int argc, char *argv[])
{
	return RLtoolsPolicy::main(argc, argv);
}
