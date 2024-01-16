#include "actuator_motors_multiplexer.hpp"

ActuatorMotorsMultiplexer::ActuatorMotorsMultiplexer() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1) {
	last_rl_tools_output_time_set = false;
	last_trigger_time_set = false;
	last_activation_time_set = false;
	use_original_controller = true;
	deactivated = false;
	overwrite = false;

	_actuator_motors_mux_pub.advertise();
	_rl_tools_multiplexer_status_pub.advertise();
}

ActuatorMotorsMultiplexer::~ActuatorMotorsMultiplexer()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ActuatorMotorsMultiplexer::init()
{
	PX4_WARN("ActuatorMotorsMultiplexer limit: %f", (double)ACTUATOR_MOTORS_MULTIPLEXER_LIMIT);
	// ScheduleOnInterval(2000_us); // 2000 us interval, 200 Hz rate
	this->init_time = hrt_absolute_time();

	// if (!_rl_tools_command_sub.registerCallback()) {
	// 	PX4_ERR("rl_tools_command_sub callback registration failed");
	// 	return false;
	// }

	if (!_actuator_motors_sub.registerCallback()) {
		PX4_ERR("actuator_motors_sub callback registration failed");
		return false;
	}
	if (!_actuator_motors_rl_tools_sub.registerCallback()) {
		PX4_ERR("actuator_motors_rl_tools_sub callback registration failed");
		return false;
	}
	return true;
}

void ActuatorMotorsMultiplexer::Run()
{
	if (should_exit()) {
		_actuator_motors_sub.unregisterCallback();
		_actuator_motors_rl_tools_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	hrt_abstime current_time = hrt_absolute_time();

	bool prev_use_original_controller = this->use_original_controller;
	bool next_use_original_controller = this->use_original_controller;

	if constexpr(TRIGGERED_BY_RC){
		if(_manual_control_input_sub.update(&manual_control_input)) {
			last_trigger_time = current_time;
			last_trigger_time_set = true;
			next_use_original_controller = manual_control_input.aux1 < 0.5f;
		}
	}
	else{
		rl_tools_policy_status_s temp;
		if(_rl_tools_policy_status_sub.update(&temp)) {
			if(temp.exit_reason == rl_tools_policy_status_s::EXIT_REASON_NONE){
				_rl_tools_policy_status = temp;
				next_use_original_controller = !_rl_tools_policy_status.active;
				last_trigger_time = current_time;
				last_trigger_time_set = true;
			}
		}
	}
	if(overwrite){
		last_trigger_time_set = true;
		last_trigger_time = current_time;
		next_use_original_controller = false;
	}

	constexpr hrt_abstime RL_TOOLS_CONTROLLER_TIMEOUT = 100*1000; // 100ms timeout
	constexpr hrt_abstime ACTIVATION_TRIGGER_TIMEOUT = 30*1000; // 200ms timeout
	next_use_original_controller = next_use_original_controller || !last_rl_tools_output_time_set || !last_trigger_time_set;
	if(last_trigger_time_set && ((current_time - last_trigger_time) > ACTIVATION_TRIGGER_TIMEOUT)){
		PX4_WARN("Activation timeout");
		next_use_original_controller = true;
		if(MODE != Mode::SWITCH_BACK){
			deactivated = true;
		}
	}
	if(last_rl_tools_output_time_set && ((current_time - last_rl_tools_output_time) > RL_TOOLS_CONTROLLER_TIMEOUT)){
		next_use_original_controller = true;
	}
	if(prev_use_original_controller != next_use_original_controller){
		if(next_use_original_controller){
			if(!deactivated){
				switch(MODE){
					case Mode::SWITCH_BACK:
						PX4_INFO("Switching to original controller");
						break;
					case Mode::TURN_OFF:
						PX4_INFO("Turning off");
						deactivated = true;
						break;
					case Mode::TURN_OFF_AFTER_TIMEOUT:
						PX4_INFO("Turning off (after button release)");
						deactivated = true;
						break;
					default:
						PX4_INFO("Turning off (unknown mode)");
						deactivated = true;
						break;
				}
			}
		}
		else{
			PX4_INFO("Switching to RLtools controller");
			last_activation_time = current_time;
			last_activation_time_set = true;
			deactivated = false;
		}
	}
	if constexpr(MODE == Mode::TURN_OFF_AFTER_TIMEOUT){
		if(!prev_use_original_controller && last_activation_time_set && ((current_time - last_activation_time) > SWITCH_BACK_TIMEOUT) && !deactivated){
			PX4_INFO("Turning off after timeout");
			deactivated = true;
		}
	}


	this->use_original_controller = next_use_original_controller;

	actuator_motors_s actuator_motors, actuator_motors_rl_tools;
	bool actuator_motors_set = false, actuator_motors_rl_tools_set = false;
	if(_actuator_motors_sub.update(&actuator_motors)){
		actuator_motors_set = true;
	}
	if(_actuator_motors_rl_tools_sub.update(&actuator_motors_rl_tools)){
		last_rl_tools_output_time_set = true;
		last_rl_tools_output_time = current_time;
		actuator_motors_rl_tools_set = true;
	}
    actuator_motors_s actuator_motors_mux;
	bool actuator_motors_mux_set = false;
	if(this->use_original_controller){
		if(actuator_motors_set){
			actuator_motors_mux = actuator_motors;
			actuator_motors_mux_set	= true;
		}
	}
	else{
		if(actuator_motors_rl_tools_set){
			actuator_motors_mux = actuator_motors_rl_tools;
			// float multiplier = 0.5;
			// for(int control_i = 0; control_i < actuator_motors_s::NUM_CONTROLS; control_i++){
			// 	if(std::isnan(actuator_motors_mux.control[control_i])){
			// 		actuator_motors_mux.control[control_i] *= multiplier;
			// 	}
			// }
			actuator_motors_mux_set	= true;
		}
	}
	if(deactivated && actuator_motors_mux_set){
		for(int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++){
			actuator_motors_mux.control[i] = NAN;
		}	
	}
	if(actuator_motors_mux_set){
		if constexpr(SCALE_OUTPUT_WITH_THROTTLE){
			for(int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++){
				float multiplier = (manual_control_input.throttle + 1)/2;
				multiplier = fmaxf(0, fminf(1, multiplier));
				actuator_motors_mux.control[i] *= multiplier;
			}	
		}
		for(int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++){
			if(!std::isnan(actuator_motors_mux.control[i])){
				actuator_motors_mux.control[i] = fminf(ACTUATOR_MOTORS_MULTIPLEXER_LIMIT, actuator_motors_mux.control[i]);
			}
		}	
		rl_tools_multiplexer_status_s status;
		status.timestamp = current_time;
		status.timestamp_sample = actuator_motors_mux.timestamp_sample;
		status.active = !this->use_original_controller;
		_actuator_motors_mux_pub.publish(actuator_motors_mux);
		_rl_tools_multiplexer_status_pub.publish(status);
	}
	perf_count(_loop_interval_perf);

}

int ActuatorMotorsMultiplexer::task_spawn(int argc, char *argv[])
{
	ActuatorMotorsMultiplexer *instance = new ActuatorMotorsMultiplexer();

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

int ActuatorMotorsMultiplexer::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int ActuatorMotorsMultiplexer::custom_command(int argc, char *argv[])
{
	if(argc > 1){
		if(strcmp(argv[0], "overwrite") == 0){
			if(strcmp(argv[1], "on") == 0){
				get_instance()->overwrite = true;
				PX4_INFO_RAW("overwrite on\n");
				return 0;
			}
			else{
				if(strcmp(argv[1], "off") == 0){
					get_instance()->overwrite = false;
					PX4_INFO_RAW("overwrite off\n");
					return 0;
				}
			}
		}
	}
	PX4_INFO_RAW("USAGE: overwrite [on/off]\n");
	return 1;
}

int ActuatorMotorsMultiplexer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLtools Multiplexer

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rl_tools_benchmark", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int actuator_motors_multiplexer_main(int argc, char *argv[])
{
	return ActuatorMotorsMultiplexer::main(argc, argv);
}
