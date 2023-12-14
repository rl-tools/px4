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
	timeout_message_sent = false;

	// controller state
	timestamp_last_forward_pass_set = false;
	controller_tick = 0;

	_actuator_motors_rl_tools_pub.advertise();
	if constexpr(MAKE_SOME_NOISE){
		_tune_control_pub.advertise();
	}
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
	ScheduleOnInterval(500_us); // 2000 us interval, 200 Hz rate
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
template <typename OBS_SPEC>
void RLtoolsPolicy::observe_rotation_matrix(rlt::Matrix<OBS_SPEC>& observation){
	// converting from FRD to FLU
    static_assert(OBS_SPEC::ROWS == 1);
    static_assert(OBS_SPEC::COLS == 18);
    float qw = +_vehicle_attitude.q[0];
    float qx = +_vehicle_attitude.q[1];
    float qy = -_vehicle_attitude.q[2];
    float qz = -_vehicle_attitude.q[3];
    rlt::set(observation, 0,  0 + 0, +(_vehicle_local_position.x - _rl_tools_command.target_position[0]));
    rlt::set(observation, 0,  0 + 1, -(_vehicle_local_position.y - _rl_tools_command.target_position[1]));
    rlt::set(observation, 0,  0 + 2, -(_vehicle_local_position.z - _rl_tools_command.target_position[2]));
    rlt::set(observation, 0,  3 + 0, (1 - 2*qy*qy - 2*qz*qz));
    rlt::set(observation, 0,  3 + 1, (    2*qx*qy - 2*qw*qz));
    rlt::set(observation, 0,  3 + 2, (    2*qx*qz + 2*qw*qy));
    rlt::set(observation, 0,  3 + 3, (    2*qx*qy + 2*qw*qz));
    rlt::set(observation, 0,  3 + 4, (1 - 2*qx*qx - 2*qz*qz));
    rlt::set(observation, 0,  3 + 5, (    2*qy*qz - 2*qw*qx));
    rlt::set(observation, 0,  3 + 6, (    2*qx*qz - 2*qw*qy));
    rlt::set(observation, 0,  3 + 7, (    2*qy*qz + 2*qw*qx));
    rlt::set(observation, 0,  3 + 8, (1 - 2*qx*qx - 2*qy*qy));
    rlt::set(observation, 0, 12 + 0, +_vehicle_local_position.vx);
    rlt::set(observation, 0, 12 + 1, -_vehicle_local_position.vy);
    rlt::set(observation, 0, 12 + 2, -_vehicle_local_position.vz);
    rlt::set(observation, 0, 15 + 0, +_vehicle_angular_velocity.xyz[0]);
    rlt::set(observation, 0, 15 + 1, -_vehicle_angular_velocity.xyz[1]);
    rlt::set(observation, 0, 15 + 2, -_vehicle_angular_velocity.xyz[2]);
}
void RLtoolsPolicy::rl_tools_control(TI substep){
    auto state_rotation_matrix_input = rlt::view(device, input, rlt::matrix::ViewSpec<1, 18>{}, 0, 0);
    observe_rotation_matrix(state_rotation_matrix_input);
    auto action_history_observation = rlt::view(device, input, rlt::matrix::ViewSpec<1, ACTION_HISTORY_LENGTH * rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM>{}, 0, 18);
    for(TI step_i = 0; step_i < ACTION_HISTORY_LENGTH; step_i++){
        for(TI action_i = 0; action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM; action_i++){
            rlt::set(action_history_observation, 0, step_i * rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM + action_i, action_history[step_i][action_i]);
        }
    }
    rlt::evaluate(device, rl_tools::checkpoint::actor::model, input, output, buffers);
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
    controller_tick++;
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

	if(_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)){
		timestamp_last_angular_velocity = current_time;
		timestamp_last_angular_velocity_set = true;
	}
	if(_vehicle_local_position_sub.update(&_vehicle_local_position)){
		timestamp_last_local_position = current_time;
		timestamp_last_local_position_set = true;
	}
	if(_vehicle_attitude_sub.update(&_vehicle_attitude)){
		timestamp_last_attitude = current_time;
		timestamp_last_attitude_set = true;
	}
	if(_rl_tools_command_sub.update(&_rl_tools_command)){
		timestamp_last_command = current_time;
		timestamp_last_command_set = true;
	}

	if(!timestamp_last_angular_velocity_set || !timestamp_last_local_position_set || !timestamp_last_attitude_set || !timestamp_last_command_set){
		return;
	}

	if((current_time - timestamp_last_angular_velocity) > OBSERVATION_TIMEOUT_ANGULAR_VELOCITY){
		if(!timeout_message_sent){
			PX4_ERR("angular velocity timeout");
			timeout_message_sent = true;
		}
		return;
	}
	if((current_time - timestamp_last_local_position) > OBSERVATION_TIMEOUT_POSITION){
		if(!timeout_message_sent){
			PX4_ERR("local position timeout");
			timeout_message_sent = true;
		}
		return;
	}
	if((current_time - timestamp_last_attitude) > OBSERVATION_TIMEOUT_ATTITUDE){
		if(!timeout_message_sent){
			PX4_ERR("attitude timeout");
			timeout_message_sent = true;
		}
		return;
	}
	if((current_time - timestamp_last_command) > COMMAND_TIMEOUT){
		if(!timeout_message_sent){
			PX4_ERR("command timeout");
			timeout_message_sent = true;
		}
		return;
	}
	timeout_message_sent = false;

	if(timestamp_last_forward_pass_set){
		if((current_time - timestamp_last_forward_pass) < CONTROL_INTERVAL){
			return;
		}
	}

    TI substep = controller_tick % CONTROL_MULTIPLE;
	rl_tools_control(substep);

	controller_tick++;
	timestamp_last_forward_pass = current_time;
	timestamp_last_forward_pass_set = true;

	actuator_motors_s actuator_motors;
	for(TI action_i=0; action_i < actuator_motors_s::NUM_CONTROLS; action_i++){
		if(action_i < rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM){
			actuator_motors.control[action_i] = rlt::get(output, 0, action_i);
		}
		else{
			actuator_motors.control[action_i] = NAN;
		}
	}
	_actuator_motors_rl_tools_pub.publish(actuator_motors);
	if constexpr(MAKE_SOME_NOISE){
		tune_control_s tune_control;
		tune_control.tune_id = 0;
		tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
		tune_control.tune_override = true;
		tune_control.frequency = 2000;
		tune_control.duration = 5000;
		_tune_control_pub.publish(tune_control);

	}
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
