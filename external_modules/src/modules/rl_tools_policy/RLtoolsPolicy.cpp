#include "RLtoolsPolicy.hpp"

RLtoolsPolicy::RLtoolsPolicy(): ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl){
	rlt::malloc(device, buffers);
	rlt::malloc(device, input);
	rlt::malloc(device, output);

	this->clear_action_history();
	// node state
	timestamp_last_angular_velocity_set = false;
	timestamp_last_local_position_set = false;
	timestamp_last_attitude_set = false;
	timestamp_last_command_set = false;
	previous_command_stale = false;
	timeout_message_sent = false;
	timestamp_last_policy_frequency_check_set = false;

	// controller state
	// timestamp_last_forward_pass_set = false;
	controller_tick = 0;
	controller_tick_substep_offset = 0;

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
template <typename T>
void quaternion_multiplication(T q1[4], T q2[4], T q_res[4]){
	q_res[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	q_res[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	q_res[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	q_res[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}
template <typename T>
void quaternion_conjugate(T q[4], T q_res[4]){
	q_res[0] = +q[0];
	q_res[1] = -q[1];
	q_res[2] = -q[2];
	q_res[3] = -q[3];
}
template <typename T, int ELEMENT>
T quaternion_to_rotation_matrix(T q[4]){
	// row-major
	T qw = q[0];
	T qx = q[1];
	T qy = q[2];
	T qz = q[3];

	static_assert(ELEMENT >= 0 && ELEMENT < 9);
	if constexpr(ELEMENT == 0){
		return 1 - 2*qy*qy - 2*qz*qz;
	}
	if constexpr(ELEMENT == 1){
		return     2*qx*qy - 2*qw*qz;
	}
	if constexpr(ELEMENT == 2){
		return     2*qx*qz + 2*qw*qy;
	}
	if constexpr(ELEMENT == 3){
		return     2*qx*qy + 2*qw*qz;
	}
	if constexpr(ELEMENT == 4){
		return 1 - 2*qx*qx - 2*qz*qz;
	}
	if constexpr(ELEMENT == 5){
		return     2*qy*qz - 2*qw*qx;
	}
	if constexpr(ELEMENT == 6){
		return     2*qx*qz - 2*qw*qy;
	}
	if constexpr(ELEMENT == 7){
		return     2*qy*qz + 2*qw*qx;
	}
	if constexpr(ELEMENT == 8){
		return 1 - 2*qx*qx - 2*qy*qy;
	}
	return 0;
}

template <typename T>
void quaternion_to_rotation_matrix(T q[4], T R[9]){
	R[0] = quaternion_to_rotation_matrix<T, 0>(q);
	R[1] = quaternion_to_rotation_matrix<T, 1>(q);
	R[2] = quaternion_to_rotation_matrix<T, 2>(q);
	R[3] = quaternion_to_rotation_matrix<T, 3>(q);
	R[4] = quaternion_to_rotation_matrix<T, 4>(q);
	R[5] = quaternion_to_rotation_matrix<T, 5>(q);
	R[6] = quaternion_to_rotation_matrix<T, 6>(q);
	R[7] = quaternion_to_rotation_matrix<T, 7>(q);
	R[8] = quaternion_to_rotation_matrix<T, 8>(q);
}

template <typename T>
void rotate_vector(T R[9], T v[3], T v_rotated[3]){
	v_rotated[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
	v_rotated[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
	v_rotated[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

template <typename OBS_SPEC>
void RLtoolsPolicy::observe_rotation_matrix(rlt::Matrix<OBS_SPEC>& observation, TestObservationMode mode){
	using T = typename OBS_SPEC::T;
	// converting from FRD to FLU
    static_assert(OBS_SPEC::ROWS == 1);
    static_assert(OBS_SPEC::COLS == 18);
	T qd[4] = {1, 0, 0, 0}, Rt_inv[9];
	if(mode >= TestObservationMode::ORIENTATION){
		// FRD to FLU
		// Validating Julia code:
		// using Rotations
		// FRD2FLU = [1 0 0; 0 -1 0; 0 0 -1]
		// q = rand(UnitQuaternion)
		// q2 = UnitQuaternion(q.q.s, q.q.v1, -q.q.v2, -q.q.v3)
		// diff = q2 - FRD2FLU * q * transpose(FRD2FLU)
		// @assert sum(abs.(diff)) < 1e-10

		T qt[4], qtc[4], qr[4];
		qt[0] = +_rl_tools_command.target_orientation[0]; // conjugate to build the difference between setpoint and current
		qt[1] = +_rl_tools_command.target_orientation[1];
		qt[2] = -_rl_tools_command.target_orientation[2];
		qt[3] = -_rl_tools_command.target_orientation[3];
		quaternion_conjugate(qt, qtc);
		quaternion_to_rotation_matrix(qtc, Rt_inv);

		qr[0] = +_vehicle_attitude.q[0];
		qr[1] = +_vehicle_attitude.q[1];
		qr[2] = -_vehicle_attitude.q[2];
		qr[3] = -_vehicle_attitude.q[3];
		// qr = qt * qd
		// qd = qt' * qr
		quaternion_multiplication(qtc, qr, qd);
	}
	if(mode >= TestObservationMode::POSITION){
		T p[3], pt[3]; // FLU
		p[0] = +(_vehicle_local_position.x - _rl_tools_command.target_position[0]);
		p[1] = -(_vehicle_local_position.y - _rl_tools_command.target_position[1]);
		p[2] = -(_vehicle_local_position.z - _rl_tools_command.target_position[2]);
		rotate_vector(Rt_inv, p, pt);
		rlt::set(observation, 0,  0 + 0, clip(pt[0], MAX_POSITION_ERROR, -MAX_POSITION_ERROR));
		rlt::set(observation, 0,  0 + 1, clip(pt[1], MAX_POSITION_ERROR, -MAX_POSITION_ERROR));
		rlt::set(observation, 0,  0 + 2, clip(pt[2], MAX_POSITION_ERROR, -MAX_POSITION_ERROR));
	}
	else{
		rlt::set(observation, 0,  0 + 0, 0);
		rlt::set(observation, 0,  0 + 1, 0);
		rlt::set(observation, 0,  0 + 2, 0);
	}
    rlt::set(observation, 0,  3 + 0, quaternion_to_rotation_matrix<T, 0>(qd));
    rlt::set(observation, 0,  3 + 1, quaternion_to_rotation_matrix<T, 1>(qd));
    rlt::set(observation, 0,  3 + 2, quaternion_to_rotation_matrix<T, 2>(qd));
    rlt::set(observation, 0,  3 + 3, quaternion_to_rotation_matrix<T, 3>(qd));
    rlt::set(observation, 0,  3 + 4, quaternion_to_rotation_matrix<T, 4>(qd));
    rlt::set(observation, 0,  3 + 5, quaternion_to_rotation_matrix<T, 5>(qd));
    rlt::set(observation, 0,  3 + 6, quaternion_to_rotation_matrix<T, 6>(qd));
    rlt::set(observation, 0,  3 + 7, quaternion_to_rotation_matrix<T, 7>(qd));
    rlt::set(observation, 0,  3 + 8, quaternion_to_rotation_matrix<T, 8>(qd));
	if(mode >= TestObservationMode::LINEAR_VELOCITY){
		T v[3], vt[3];
		v[0] = +_vehicle_local_position.vx;
		v[1] = -_vehicle_local_position.vy;
		v[2] = -_vehicle_local_position.vz;
		rotate_vector(Rt_inv, v, vt);
		rlt::set(observation, 0, 12 + 0, clip(vt[0], MAX_VELOCITY_ERROR, -MAX_VELOCITY_ERROR));
		rlt::set(observation, 0, 12 + 1, clip(vt[1], MAX_VELOCITY_ERROR, -MAX_VELOCITY_ERROR));
		rlt::set(observation, 0, 12 + 2, clip(vt[2], MAX_VELOCITY_ERROR, -MAX_VELOCITY_ERROR));
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
void RLtoolsPolicy::clear_action_history(){
	for(TI step_i = 0; step_i < ACTION_HISTORY_LENGTH; step_i++){
		for(TI action_i = 0; action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM; action_i++){
			action_history[step_i][action_i] = rl_tools::checkpoint::meta::action_history_init;
		}
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
	status.control_interval = NAN;

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

	constexpr bool PUBLISH_NON_COMPLETE_STATUS = false;
	if(!angular_velocity_update){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE;
		if constexpr(PUBLISH_NON_COMPLETE_STATUS){
			_rl_tools_policy_status_pub.publish(status);
		}
		return;
	}

	if(!timestamp_last_angular_velocity_set || !timestamp_last_local_position_set || !timestamp_last_attitude_set){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_NOT_ALL_OBSERVATIONS_SET;
		if constexpr(PUBLISH_NON_COMPLETE_STATUS){
			_rl_tools_policy_status_pub.publish(status);
		}
		return;
	}

	if((current_time - timestamp_last_angular_velocity) > OBSERVATION_TIMEOUT_ANGULAR_VELOCITY){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_ANGULAR_VELOCITY_STALE;
		if constexpr(PUBLISH_NON_COMPLETE_STATUS){
			_rl_tools_policy_status_pub.publish(status);
		}
		if(!timeout_message_sent){
			PX4_ERR("angular velocity timeout");
			timeout_message_sent = true;
		}
		return;
	}
	if((current_time - timestamp_last_local_position) > OBSERVATION_TIMEOUT_POSITION){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_LOCAL_POSITION_STALE;
		if constexpr(PUBLISH_NON_COMPLETE_STATUS){
			_rl_tools_policy_status_pub.publish(status);
		}
		if(!timeout_message_sent){
			PX4_ERR("local position timeout");
			timeout_message_sent = true;
		}
		return;
	}
	if((current_time - timestamp_last_attitude) > OBSERVATION_TIMEOUT_ATTITUDE){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_ATTITUDE_STALE;
		if constexpr(PUBLISH_NON_COMPLETE_STATUS){
			_rl_tools_policy_status_pub.publish(status);
		}
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
		if(previous_command_stale){
			clear_action_history();	
			controller_tick_substep_offset = controller_tick % CONTROL_MULTIPLE;
		}
		status.command_stale = false;
	}
	status.active = !status.command_stale && _rl_tools_command.active;
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

    TI substep = (controller_tick-controller_tick_substep_offset) % CONTROL_MULTIPLE;
	status.substep = substep;
	T policy_interval = perf_mean(_loop_interval_policy_perf);
	perf_count(_loop_interval_policy_perf);
	status.control_interval = policy_interval;
	rl_tools_control(substep, TEST_OBSERVATION_MODE);
	if(!timestamp_last_policy_frequency_check_set || (current_time - timestamp_last_policy_frequency_check) > POLICY_FREQUENCY_CHECK_INTERVAL){
		if(timestamp_last_policy_frequency_check_set){
			T policy_interval_target_diff = policy_interval * 1e6 - CONTROL_INTERVAL;
			if(fabs(policy_interval_target_diff) > POLICY_INTERVAL_WARNING_THRESHOLD){
				PX4_WARN("Policy frequency deviation: %.2fHz (target %.2fHz)", (double)1/policy_interval, (double)1e6/CONTROL_INTERVAL);
			}
		}
		timestamp_last_policy_frequency_check = current_time;
		timestamp_last_policy_frequency_check_set = true;
	}

	rl_tools_policy_input_s input_msg;
	constexpr TI STATE_OBSERVATION_DIM = 18;
	static_assert(rl_tools_policy_input_s::STATE_OBSERVATION_DIM == STATE_OBSERVATION_DIM);
	input_msg.timestamp = current_time;
	input_msg.timestamp_sample = current_time;
	for(TI state_i = 0; state_i < STATE_OBSERVATION_DIM; state_i++){
		input_msg.state_observation[state_i] = rlt::get(input, 0, state_i);
	}

	_rl_tools_policy_input_pub.publish(input_msg);
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
			// todo: add simulation check
			constexpr T training_min = rl_tools::checkpoint::meta::action_limit_lower;
			constexpr T training_max = rl_tools::checkpoint::meta::action_limit_upper;
			T scaled_value = (training_max - training_min) * value + training_min;
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
	perf_print_counter(_loop_interval_policy_perf);
	PX4_INFO_RAW("Checkpoint: %s\n", rl_tools::checkpoint::meta::name);
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
