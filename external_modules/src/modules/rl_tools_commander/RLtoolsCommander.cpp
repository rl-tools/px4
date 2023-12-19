#include "RLtoolsCommander.hpp"

RLtoolsCommander::RLtoolsCommander() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1) {
	command_active = false;
	last_rc_update_time_set = false;
	last_position_update_time_set = false;
	activation_position[0] = 0;
	activation_position[1] = 1;
	activation_position[2] = 2;
	_rl_tools_command_pub.advertise();
	if constexpr(MAKE_SOME_NOISE){
		_tune_control_pub.advertise();
	}
}

RLtoolsCommander::~RLtoolsCommander(){
	perf_free(_loop_interval_perf);
}

bool RLtoolsCommander::init()
{
	// ScheduleOnInterval(2000_us); // 2000 us interval, 200 Hz rate

	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position_sub callback registration failed");
		return false;
	}
	return true;
}

void RLtoolsCommander::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	uint32_t current_time = hrt_absolute_time();

	bool prev_command_active = command_active;
	bool next_command_active = command_active;

	{
		manual_control_setpoint_s manual_control_input;
		if(_manual_control_input_sub.update(&manual_control_input)) {
			last_rc_update_time_set = true;
			last_rc_update_time = current_time;
			next_command_active = manual_control_input.aux1 >= 0.5f;
		}
	}
	{
		if(_vehicle_local_position_sub.update(&vehicle_local_position)) {
			last_position_update_time_set = true;
			last_position_update_time = current_time;
		}
	}

	constexpr uint32_t POSITION_TIMEOUT = 1000*1000; // 100ms timeout
	constexpr uint32_t RC_TRIGGER_TIMEOUT = 2000*1000; // 200ms timeout
	next_command_active = next_command_active && last_rc_update_time_set && last_position_update_time_set;
	if(last_rc_update_time_set && ((current_time - last_rc_update_time) > RC_TRIGGER_TIMEOUT)){
		next_command_active = false;
	}
	if(last_position_update_time_set && ((current_time - last_position_update_time) > POSITION_TIMEOUT)){
		next_command_active = false;
	}
	if(prev_command_active != next_command_active){
		if(next_command_active){ 
			PX4_INFO("Command enabled");
			activation_position[0] = vehicle_local_position.x;
			activation_position[1] = vehicle_local_position.y;
			activation_position[2] = vehicle_local_position.z - TARGET_HEIGHT; // FRD
			if constexpr(MAKE_SOME_NOISE){
				if(next_command_active){
					tune_control_s tune_control;
					tune_control.tune_id = 1;
					tune_control.volume = 100; //tune_control_s::VOLUME_LEVEL_DEFAULT;
					tune_control.tune_override = true;
					tune_control.frequency = 2000;
					tune_control.duration = 10000;
					_tune_control_pub.publish(tune_control);
				}
			}
		}
		else{
			PX4_INFO("Command disabled");
		}
	}

	command_active = next_command_active;

	if(command_active){
		rl_tools_command_s command;
		command.timestamp = current_time;
		command.timestamp_sample = current_time;
		command.target_position[0] = activation_position[0];
		command.target_position[1] = activation_position[1];
		command.target_position[2] = activation_position[2];
		_rl_tools_command_pub.publish(command);
	}
	perf_count(_loop_interval_perf);
}

int RLtoolsCommander::task_spawn(int argc, char *argv[])
{
	RLtoolsCommander *instance = new RLtoolsCommander();

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

int RLtoolsCommander::print_status()
{
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int RLtoolsCommander::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RLtoolsCommander::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLtools Commander

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rl_tools_benchmark", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rl_tools_commander_main(int argc, char *argv[])
{
	return RLtoolsCommander::main(argc, argv);
}
