#include "RLtoolsPolicy.hpp"
#undef OK

RLtoolsPolicy::RLtoolsPolicy(): ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl){
	// node state
	timestamp_last_angular_velocity_set = false;
	timestamp_last_local_position_set = false;
	timestamp_last_attitude_set = false;
	timestamp_last_command_set = false;
	previous_command_stale = false;
	previous_active = false;
	timeout_message_sent = false;
	timestamp_last_policy_frequency_check_set = false;
	last_intermediate_status_set = false;
	last_native_status_set = false;
	policy_frequency_check_counter = 0;

	_actuator_motors_rl_tools_pub.advertise();
	_tune_control_pub.advertise();
}
void RLtoolsPolicy::reset(){
	this->visual_odometry_stale_counter = 0;
	this->timestamp_last_visual_odometry_stale_set = false;
	for(int action_i=0; action_i < RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM; action_i++){
		this->previous_action[action_i] = RESET_PREVIOUS_ACTION_VALUE;
	}
	rl_tools_inference_applications_l2f_reset();
}

RLtoolsPolicy::~RLtoolsPolicy()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool RLtoolsPolicy::init()
{
	this->init_time = hrt_absolute_time();
	// ScheduleOnInterval(500_us); // 2000 us interval, 200 Hz rate
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position_sub callback registration failed");
		return false;
	}
	if (!_vehicle_visual_odometry_sub.registerCallback()) {
		PX4_ERR("vehicle_visual_odometry_sub callback registration failed");
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
	PX4_INFO("Checkpoint: %s", rl_tools_inference_applications_l2f_checkpoint_name());

	RLtoolsInferenceApplicationsL2FAction action;
	float abs_diff = rl_tools_inference_applications_l2f_test(&action);
	PX4_INFO("Checkpoint test diff: %f", abs_diff);
	for(TI output_i = 0; output_i < RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM; output_i++){
		PX4_INFO("output[%d]: %f", output_i, action.action[output_i]);
	}

	constexpr float EPSILON = 1e-5;

	bool healthy = abs_diff < EPSILON;
	if(!healthy){
		PX4_ERR("Checkpoint test failed with diff %.10f", abs_diff);
	}
	else{
		PX4_INFO("Checkpoint test passed with diff %.10f", abs_diff);
	}
	return healthy;
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

void RLtoolsPolicy::observe(RLtoolsInferenceApplicationsL2FObservation& observation, TestObservationMode mode){
	// converting from FRD to FLU
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

		observation.orientation[0] = qd[0];
		observation.orientation[1] = qd[1];
		observation.orientation[2] = qd[2];
		observation.orientation[3] = qd[3];
	}
	else{
		observation.orientation[0] = 1;
		observation.orientation[1] = 0;
		observation.orientation[2] = 0;
		observation.orientation[3] = 0;
	}
	if(mode >= TestObservationMode::POSITION){
		T p[3], pt[3]; // FLU
		p[0] = +(position[0] - _rl_tools_command.target_position[0]);
		p[1] = -(position[1] - _rl_tools_command.target_position[1]);
		p[2] = -(position[2] - _rl_tools_command.target_position[2]);
		rotate_vector(Rt_inv, p, pt); // The position and velocity error are in the target frame
		observation.position[0] = clip(pt[0], max_position_error, -max_position_error);
		observation.position[1] = clip(pt[1], max_position_error, -max_position_error);
		observation.position[2] = clip(pt[2], max_position_error, -max_position_error);
	}
	else{
		observation.position[0] = 0;
		observation.position[1] = 0;
		observation.position[2] = 0;
	}
	if(mode >= TestObservationMode::LINEAR_VELOCITY){
		T v[3], vt[3];
		v[0] = +(linear_velocity[0] - _rl_tools_command.target_linear_velocity[0]);
		v[1] = -(linear_velocity[1] - _rl_tools_command.target_linear_velocity[1]);
		v[2] = -(linear_velocity[2] - _rl_tools_command.target_linear_velocity[2]);
		rotate_vector(Rt_inv, v, vt);
		observation.linear_velocity[0] = clip(vt[0], max_velocity_error, -max_velocity_error);
		observation.linear_velocity[1] = clip(vt[1], max_velocity_error, -max_velocity_error);
		observation.linear_velocity[2] = clip(vt[2], max_velocity_error, -max_velocity_error);
	}
	else{
		observation.linear_velocity[0] = 0;
		observation.linear_velocity[1] = 0;
		observation.linear_velocity[2] = 0;
	}
	if(mode >= TestObservationMode::ANGULAR_VELOCITY){
		observation.angular_velocity[0] = +_vehicle_angular_velocity.xyz[0];
		observation.angular_velocity[1] = -_vehicle_angular_velocity.xyz[1];
		observation.angular_velocity[2] = -_vehicle_angular_velocity.xyz[2];
	}
	else{
		observation.angular_velocity[0] = 0;
		observation.angular_velocity[1] = 0;
		observation.angular_velocity[2] = 0;
	}
	for(int action_i=0; action_i < RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM; action_i++){
		observation.previous_action[action_i] = this->previous_action[action_i];
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
	hrt_abstime current_time = hrt_absolute_time();

	rl_tools_policy_status_s status;
	status.timestamp = current_time;
	status.timestamp_sample = current_time;
	status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_NONE;
	status.substep = 0;
	status.active = false;
	status.control_interval = NAN;
	status.visual_odometry_stale_counter = this->visual_odometry_stale_counter;
	status.visual_odometry_age = 0;
	status.visual_odometry_dt_mean = 0;
	status.visual_odometry_dt_std = 0;
	for(TI odometry_dt_i = 0; odometry_dt_i < rl_tools_policy_status_s::NUM_VISUAL_ODOMETRY_DT_MAX; odometry_dt_i++){
		status.visual_odometry_dt_max[odometry_dt_i] = 0;
	}

	if(RLtoolsPolicy::ODOMETRY_SOURCE == RLtoolsPolicy::OdometrySource::LOCAL_POSITION){
		status.odometry_source = rl_tools_policy_status_s::ODOMETRY_SOURCE_LOCAL_POSITION;
	}
	else{
		if(RLtoolsPolicy::ODOMETRY_SOURCE == RLtoolsPolicy::OdometrySource::VISUAL_ODOMETRY){
			status.odometry_source = rl_tools_policy_status_s::ODOMETRY_SOURCE_VISUAL_ODOMETRY;
		}
		else{
			status.odometry_source = rl_tools_policy_status_s::ODOMETRY_SOURCE_UNKNOWN;
		}
	}

	bool angular_velocity_update = false;
	if(status.subscription_update_angular_velocity = _vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)){
		timestamp_last_angular_velocity = current_time;
		timestamp_last_angular_velocity_set = true;
		angular_velocity_update = true;
	}
	status.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	if(status.subscription_update_local_position = _vehicle_local_position_sub.update(&_vehicle_local_position)){
		timestamp_last_local_position = current_time;
		timestamp_last_local_position_set = true;
	}
	if(status.subscription_update_visual_odometry = _vehicle_visual_odometry_sub.update(&_vehicle_visual_odometry)){
		if(timestamp_last_visual_odometry_set){
			odometry_dts[odometry_dt_index] = current_time - timestamp_last_visual_odometry;
			odometry_dt_index = (odometry_dt_index + 1) % NUM_ODOMETRY_DTS;
			if(odometry_dt_index == 0){
				odometry_dts_full = true;
			}
		}
		timestamp_last_visual_odometry = current_time;
		timestamp_last_visual_odometry_set = true;
	}
	for(TI odometry_dt_i = 0; odometry_dt_i < (odometry_dts_full ? NUM_ODOMETRY_DTS : odometry_dt_index); odometry_dt_i++){
		auto value = odometry_dts[odometry_dt_i];
		status.visual_odometry_dt_mean += value;
		status.visual_odometry_dt_std += value * value;
		TI max_index = 0;
		bool max_index_set = false;
		for(TI odometry_dt_max_i = 0; odometry_dt_max_i < rl_tools_policy_status_s::NUM_VISUAL_ODOMETRY_DT_MAX; odometry_dt_max_i++){
			if(value > status.visual_odometry_dt_max[odometry_dt_max_i]){
				max_index = odometry_dt_max_i;
				max_index_set = true;
			}
		}
		if(max_index_set){
			status.visual_odometry_dt_max[max_index] = value;
		}
	}
	status.visual_odometry_dt_mean /= NUM_ODOMETRY_DTS;
	status.visual_odometry_dt_std = sqrt(status.visual_odometry_dt_std / NUM_ODOMETRY_DTS - status.visual_odometry_dt_mean * status.visual_odometry_dt_mean);

	if(status.subscription_update_attitude = _vehicle_attitude_sub.update(&_vehicle_attitude)){
		timestamp_last_attitude = current_time;
		timestamp_last_attitude_set = true;
	}
	if(status.subscription_update_rl_tools_command = _rl_tools_command_sub.update(&_rl_tools_command)){
		timestamp_last_command = current_time;
		timestamp_last_command_set = true;
		previous_command_stale = false;
	}

	// if(_manual_control_input_sub.update(&_manual_control_input)) {
	// 	timestamp_last_manual_control_input_set = true;
	// 	timestamp_last_manual_control_input = current_time;
	// }

	constexpr bool PUBLISH_NON_COMPLETE_STATUS = true;
	if(!angular_velocity_update){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE;
		if constexpr(PUBLISH_NON_COMPLETE_STATUS){
			_rl_tools_policy_status_pub.publish(status);
		}
		return;
	}

	bool timestamp_last_odometry_set = RLtoolsPolicy::ODOMETRY_SOURCE == RLtoolsPolicy::OdometrySource::LOCAL_POSITION && timestamp_last_local_position_set || RLtoolsPolicy::ODOMETRY_SOURCE == RLtoolsPolicy::OdometrySource::VISUAL_ODOMETRY && timestamp_last_visual_odometry_set;
	if(!timestamp_last_angular_velocity_set || !timestamp_last_odometry_set || !timestamp_last_attitude_set){
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
	if(RLtoolsPolicy::ODOMETRY_SOURCE == RLtoolsPolicy::OdometrySource::LOCAL_POSITION){
		if((current_time - timestamp_last_local_position) > OBSERVATION_TIMEOUT_LOCAL_POSITION){
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
		else{
			position[0] = _vehicle_local_position.x;
			position[1] = _vehicle_local_position.y;
			position[2] = _vehicle_local_position.z;
			linear_velocity[0] = _vehicle_local_position.vx;
			linear_velocity[1] = _vehicle_local_position.vy;
			linear_velocity[2] = _vehicle_local_position.vz;
		}
	}
	else{
		if(RLtoolsPolicy::ODOMETRY_SOURCE == RLtoolsPolicy::OdometrySource::VISUAL_ODOMETRY){
			status.visual_odometry_age = current_time - timestamp_last_visual_odometry;
			if((current_time - timestamp_last_visual_odometry) > OBSERVATION_TIMEOUT_VISUAL_ODOMETRY){
				if(!timestamp_last_visual_odometry_stale_set || timestamp_last_visual_odometry_stale != timestamp_last_visual_odometry){
					// rising edge
					visual_odometry_stale_counter++;
					tune_control_s tune_control;
					tune_control.timestamp = current_time;
					tune_control.tune_id = 0;
					tune_control.volume = 100; //tune_control_s::VOLUME_LEVEL_DEFAULT;
					tune_control.tune_override = true;
					tune_control.frequency = 1000;
					tune_control.duration = 10000;
					_tune_control_pub.publish(tune_control);
					PX4_WARN("VISUAL ODOMETRY STALE: Begin");
				}
				timestamp_last_visual_odometry_stale = timestamp_last_visual_odometry;
				timestamp_last_visual_odometry_stale_set = true;
				status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_VISUAL_ODOMETRY_STALE;

				if constexpr(PUBLISH_NON_COMPLETE_STATUS){
					_rl_tools_policy_status_pub.publish(status);
				}
				if(!timeout_message_sent){
					PX4_ERR("Visual odometry timeout");
					timeout_message_sent = true;
				}
				return;
			}
			else{
				if(timestamp_last_visual_odometry_stale_set){
					// falling edge
					auto diff = current_time - timestamp_last_visual_odometry_stale;
					tune_control_s tune_control;
					tune_control.timestamp = current_time;
					tune_control.tune_id = 0;
					tune_control.volume = 100; //tune_control_s::VOLUME_LEVEL_DEFAULT;
					tune_control.tune_override = true;
					tune_control.frequency = 2000;
					tune_control.duration = min(10000000, diff);
					_tune_control_pub.publish(tune_control);
					PX4_WARN("VISUAL ODOMETRY STALE: End %llu uS", diff);
				}
				timestamp_last_visual_odometry_stale_set = false;
				if(_vehicle_visual_odometry.pose_frame != vehicle_odometry_s::POSE_FRAME_NED){
					status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_VISUAL_ODOMETRY_WRONG_FRAME;
					if constexpr(PUBLISH_NON_COMPLETE_STATUS){
						_rl_tools_policy_status_pub.publish(status);
					}
					if(!timeout_message_sent){
						PX4_ERR("Visual odometry wrong frame (should be NED)");
						timeout_message_sent = true;
					}
					return;
				}
				else{
					bool position_nan = std::isnan(_vehicle_visual_odometry.position[0]) || std::isnan(_vehicle_visual_odometry.position[1]) || std::isnan(_vehicle_visual_odometry.position[2]);
					bool velocity_nan = std::isnan(_vehicle_visual_odometry.velocity[0]) || std::isnan(_vehicle_visual_odometry.velocity[1]) || std::isnan(_vehicle_visual_odometry.velocity[2]);
					if(position_nan || velocity_nan){
						status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_VISUAL_ODOMETRY_NAN;
						if constexpr(PUBLISH_NON_COMPLETE_STATUS){
							_rl_tools_policy_status_pub.publish(status);
						}
						if(!timeout_message_sent){
							PX4_ERR("Visual odometry contains NANs in position or velocity");
							timeout_message_sent = true;
						}
						return;
					}
					else{
						position[0] = _vehicle_visual_odometry.position[0];
						position[1] = _vehicle_visual_odometry.position[1];
						position[2] = _vehicle_visual_odometry.position[2];
						linear_velocity[0] = _vehicle_visual_odometry.velocity[0];
						linear_velocity[1] = _vehicle_visual_odometry.velocity[1];
						linear_velocity[2] = _vehicle_visual_odometry.velocity[2];
					}
				}
			}
		}
	}
	// position and linear_velocity are guaranteed to be set after this point

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
	timeout_message_sent = false;

	if(!timestamp_last_command_set || (current_time - timestamp_last_command) > COMMAND_TIMEOUT){
		status.command_stale = true;
		if(!previous_command_stale){
			PX4_WARN("Command turned stale at: %f %f %f", position[0], position[1], position[2]);
			_rl_tools_command.target_position[0] = position[0];
			_rl_tools_command.target_position[1] = position[1];
			_rl_tools_command.target_position[2] = position[2];
			_rl_tools_command.target_linear_velocity[0] = 0;
			_rl_tools_command.target_linear_velocity[1] = 0;
			_rl_tools_command.target_linear_velocity[2] = 0;
		}
		previous_command_stale = true;
	}
	else{
		status.command_stale = false;
	}


	RLtoolsInferenceApplicationsL2FObservation observation;
	RLtoolsInferenceApplicationsL2FAction action;
	observe(observation, TEST_OBSERVATION_MODE);
	auto executor_status = rl_tools_inference_applications_l2f_control(current_time * 1000, &observation, &action);

	if(executor_status.source != RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL){
		status.exit_reason = rl_tools_policy_status_s::EXIT_REASON_EXECUTOR_STATUS_SOURCE_NOT_CONTROL;
		if constexpr(PUBLISH_NON_COMPLETE_STATUS){
			_rl_tools_policy_status_pub.publish(status);
		}
		return;
	}

	bool next_active = !status.command_stale && _rl_tools_command.active;
	if(!previous_active && next_active){
		this->reset();
		PX4_INFO("Resetting Inference Executor (Recurrent State)");
	}
	status.active = next_active;


	// no return after this point!

	rl_tools_policy_input_s input_msg;
	input_msg.active = status.active;
	static_assert(rl_tools_policy_input_s::ACTION_DIM == RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM);
	input_msg.timestamp = current_time;
	input_msg.timestamp_sample = current_time;
	input_msg.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	for(TI dim_i = 0; dim_i < 3; dim_i++){
		input_msg.position[dim_i] = observation.position[dim_i];
		input_msg.orientation[dim_i] = observation.orientation[dim_i];
		input_msg.linear_velocity[dim_i] = observation.linear_velocity[dim_i];
		input_msg.angular_velocity[dim_i] = observation.angular_velocity[dim_i];
	}
	input_msg.orientation[3] = observation.orientation[3];
	for(TI dim_i = 0; dim_i < RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM; dim_i++){
		input_msg.previous_action[dim_i] = observation.previous_action[dim_i];
	}

	_rl_tools_policy_input_pub.publish(input_msg);
	_rl_tools_policy_status_pub.publish(status);

	actuator_motors_s actuator_motors = {}; // zero initialize to set e.g. the reversible_flags to all 0
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	for(TI action_i=0; action_i < actuator_motors_s::NUM_CONTROLS; action_i++){
		if(action_i < RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM){
			T value = action.action[action_i];
			this->previous_action[action_i] = value;
			value = (value + 1) / 2;
			// T scaled_value = value * (SCALE_OUTPUT_WITH_THROTTLE ? (_manual_control_input.throttle + 1)/2 : 0.5);
			// todo: add simulation check
			constexpr T training_min = 0; //rl_tools::checkpoint::meta::action_limit_lower;
			constexpr T training_max = 1.0; //rl_tools::checkpoint::meta::action_limit_upper;
			T scaled_value = (training_max - training_min) * value + training_min;
			actuator_motors.control[action_i] = scaled_value;
		}
		else{
			actuator_motors.control[action_i] = NAN;
		}
	}
	if constexpr(RLtoolsPolicy::REMAP_FROM_CRAZYFLIE){
		actuator_motors_s temp = actuator_motors;
		temp.control[0] = actuator_motors.control[0];
		temp.control[1] = actuator_motors.control[2];
		temp.control[2] = actuator_motors.control[3];
		temp.control[3] = actuator_motors.control[1];
		actuator_motors = temp;
	}
	_actuator_motors_rl_tools_pub.publish(actuator_motors);
	perf_end(_loop_perf);
	previous_active = next_active;

	if(executor_status.source == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL){
		if(executor_status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_INTERMEDIATE){
			this->last_intermediate_status = executor_status;
			this->last_intermediate_status_set = true;
		}
		else if(executor_status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_NATIVE){
			this->last_native_status = executor_status;
			this->last_native_status_set = true;
		}
	}

	if(!this->timestamp_last_policy_frequency_check_set || (current_time - timestamp_last_policy_frequency_check) > POLICY_FREQUENCY_CHECK_INTERVAL){
		if(this->timestamp_last_policy_frequency_check_set){
			if(last_intermediate_status_set){
				if(!this->last_intermediate_status.timing_bias.OK || !this->last_intermediate_status.timing_jitter.OK){
					PX4_WARN("RLtoolsPolicy: INTERMEDIATE: BIAS %fx JITTER %fx", this->last_intermediate_status.timing_bias.MAGNITUDE, this->last_intermediate_status.timing_jitter.MAGNITUDE);
				}
				else{
					if(this->policy_frequency_check_counter % POLICY_FREQUENCY_INFO_INTERVAL == 0){
						PX4_INFO("RLtoolsPolicy: INTERMEDIATE: BIAS %fx JITTER %fx", this->last_intermediate_status.timing_bias.MAGNITUDE, this->last_intermediate_status.timing_jitter.MAGNITUDE);
					}
				}
			}
			if(last_native_status_set){
				if(!this->last_native_status.timing_bias.OK || !this->last_native_status.timing_jitter.OK){
					PX4_WARN("RLtoolsPolicy: NATIVE: BIAS %fx JITTER %fx", this->last_native_status.timing_bias.MAGNITUDE, this->last_native_status.timing_jitter.MAGNITUDE);
				}
				else{
					if(this->policy_frequency_check_counter % POLICY_FREQUENCY_INFO_INTERVAL == 0){
						PX4_INFO("RLtoolsPolicy: NATIVE: BIAS %fx JITTER %fx", this->last_native_status.timing_bias.MAGNITUDE, this->last_native_status.timing_jitter.MAGNITUDE);
					}
				}
			}
		}
		this->num_healthy_executor_statii_intermediate = 0;
		this->num_non_healthy_executor_statii_intermediate = 0;
		this->num_healthy_executor_statii_native = 0;
		this->num_non_healthy_executor_statii_native = 0;
		this->num_statii = 0;
		this->timestamp_last_policy_frequency_check = current_time;
		this->timestamp_last_policy_frequency_check_set = true;
		this->policy_frequency_check_counter++;
	}
	this->num_statii++;
	this->num_healthy_executor_statii_intermediate += executor_status.OK && executor_status.source == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL && executor_status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_INTERMEDIATE;
	this->num_non_healthy_executor_statii_intermediate += (!executor_status.OK) && executor_status.source == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL && executor_status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_INTERMEDIATE;
	this->num_healthy_executor_statii_native += executor_status.OK && executor_status.source == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL && executor_status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_NATIVE;
	this->num_non_healthy_executor_statii_native += (!executor_status.OK) && executor_status.source == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_SOURCE_CONTROL && executor_status.step_type == RL_TOOLS_INFERENCE_EXECUTOR_STATUS_STEP_TYPE_NATIVE;
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
	PX4_INFO_RAW("Checkpoint: %s\n", rl_tools_inference_applications_l2f_checkpoint_name());
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
