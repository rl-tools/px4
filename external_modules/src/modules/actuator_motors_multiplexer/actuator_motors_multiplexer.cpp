#include "actuator_motors_multiplexer.hpp"

ActuatorMotorsMultiplexer::ActuatorMotorsMultiplexer() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {
	_actuator_motors_mux_pub.advertise();
}

ActuatorMotorsMultiplexer::~ActuatorMotorsMultiplexer()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ActuatorMotorsMultiplexer::init()
{
	ScheduleOnInterval(2000_us); // 2000 us interval, 200 Hz rate
	this->init_time = hrt_absolute_time();

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

	uint32_t current_time = hrt_absolute_time();

	bool prev_use_original_controller = this->use_original_controller;
	bool next_use_original_controller = this->use_original_controller;

	{
		manual_control_setpoint_s manual_control_input;
		if(_manual_control_input_sub.update(&manual_control_input)) {
			last_rc_update_time_set = true;
			last_rc_update_time = current_time;
			next_use_original_controller = manual_control_input.aux1 < 0.5f;
		}
	}

	constexpr uint32_t RL_TOOLS_CONTROLLER_TIMEOUT = 100*1000; // 100ms timeout
	constexpr uint32_t RC_TRIGGER_TIMEOUT = 2000*1000; // 200ms timeout
	next_use_original_controller = next_use_original_controller || !last_rl_tools_output_time_set || !last_rc_update_time_set;
	if(last_rc_update_time_set && ((current_time - last_rc_update_time) > RC_TRIGGER_TIMEOUT)){
		next_use_original_controller = true;
	}
	if(last_rl_tools_output_time_set && ((current_time - last_rl_tools_output_time) > RL_TOOLS_CONTROLLER_TIMEOUT)){
		next_use_original_controller = true;
	}
	if(prev_use_original_controller != next_use_original_controller){
		if(next_use_original_controller){
			PX4_INFO("Switching to original controller");
		}
		else{
			PX4_INFO("Switching to RLtools controller");
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
	if(this->use_original_controller){
		if(actuator_motors_set){
			_actuator_motors_mux_pub.publish(actuator_motors);
		}
	}
	else{
		if(actuator_motors_rl_tools_set){
			_actuator_motors_mux_pub.publish(actuator_motors_rl_tools);
		}
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
	return print_usage("unknown command");
}

int ActuatorMotorsMultiplexer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLtools Benchmark

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
