#include "RLtoolsCommander.hpp"

RLtoolsCommander::RLtoolsCommander() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {
	command_active = false;
	last_rc_update_time_set = false;
	last_position_update_time_set = false;
	last_attitude_update_time_set = false;
	activation_position[0] = 0;
	activation_position[1] = 0;
	activation_position[2] = 0;
	activation_orientation[0] = 1;
	activation_orientation[1] = 0;
	activation_orientation[2] = 0;
	activation_orientation[3] = 0;
	_rl_tools_command_pub.advertise();
	if constexpr(MAKE_SOME_NOISE){
		_tune_control_pub.advertise();
	}
	target_height = DEFAULT_TARGET_HEIGHT;
	overwrite = false;
	mode = DEFAULT_MODE;
	trajectory.initialized = false;
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
void RLtoolsCommander::parameters_update(){
	auto activ_src_pre = _rlt_activ_src.get();
	auto activ_btn_pre = _rlt_activ_btn.get();
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
	if (activ_src_pre != _rlt_activ_src.get() || activ_btn_pre != _rlt_activ_btn.get()){
		PX4_INFO("Parameter updated: activ_src=%d, activ_btn=%d", (int)_rlt_activ_src.get(), (int)_rlt_activ_btn.get());
	}
}

void RLtoolsCommander::FigureEight::update(hrt_abstime now){
	if(!initialized){
		reset(now);
	}
	float t = (now - start_time) / 1000000.0f;
	float dt = (now - last_update_time) / 1000000.0f;
	float target_speed = 1/interval;
	float speed = target_speed;
	if(t < warmup_time){
		speed = target_speed * t/warmup_time;
	}
	progress += dt * speed;
	float pi = M_PI;
	position[0] = cosf(progress*2.0f*pi + pi / 2.0f) * scale;
	linear_velocity[0] = -sinf(progress*2*pi + pi / 2) * scale * 2 * pi * speed;
	position[1] = sinf(2*(progress*2*pi + pi / 2)) / 2.0f * scale;
	linear_velocity[1] = cosf(2*(progress*2*pi + pi / 2)) / 2.0f * scale * 4 * pi * speed;
	position[2] = 0;
	linear_velocity[2] = 0;
	last_update_time = now;
}

void RLtoolsCommander::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	this->parameters_update();

	hrt_abstime current_time = hrt_absolute_time();

	bool prev_command_active = command_active;
	bool next_command_active = command_active;

	{
		manual_control_setpoint_s manual_control_input;
		if(_manual_control_input_sub.update(&manual_control_input)) {
			last_rc_update_time_set = true;
			last_rc_update_time = current_time;
			if(_rlt_activ_src.get() == 0){
				next_command_active = manual_control_input.aux1 >= 0.5f;
			}
			else{
				next_command_active = (manual_control_input.buttons & (1 << _rlt_activ_btn.get())) != 0;
			}
		}
	}
	if(overwrite){
		next_command_active = true;
		last_rc_update_time_set = true;
		last_rc_update_time = current_time;
	}
	{
		trajectory_setpoint_s temp_trajectory_setpoint;
		if(_trajectory_setpoint_sub.update(&temp_trajectory_setpoint)) {
			if(
				PX4_ISFINITE(temp_trajectory_setpoint.position[0]) &&
				PX4_ISFINITE(temp_trajectory_setpoint.position[1]) &&
				PX4_ISFINITE(temp_trajectory_setpoint.position[2]) &&
				PX4_ISFINITE(temp_trajectory_setpoint.velocity[0]) &&
				PX4_ISFINITE(temp_trajectory_setpoint.velocity[1]) &&
				PX4_ISFINITE(temp_trajectory_setpoint.velocity[2])
			){
				last_trajectory_setpoint_update_time_set = true;
				last_trajectory_setpoint_update_time = trajectory_setpoint.timestamp;
				trajectory_setpoint = temp_trajectory_setpoint;
			}
		}
	}
	{
		if(_vehicle_local_position_sub.update(&vehicle_local_position)) {
			last_position_update_time_set = true;
			last_position_update_time = current_time;
		}
	}
	{
		if(_vehicle_attitude_sub.update(&vehicle_attitude)) {
			last_attitude_update_time_set = true;
			last_attitude_update_time = current_time;
		}
	}


	constexpr hrt_abstime POSITION_TIMEOUT = 100*1000; // 100ms timeout
	constexpr hrt_abstime ATTITUDE_TIMEOUT = 100*1000; // 100ms timeout
	constexpr hrt_abstime RC_TRIGGER_TIMEOUT = 200*1000; // 200ms timeout
	constexpr hrt_abstime OFFBOARD_TIMEOUT = 200*1000; // 200ms timeout
	// next_command_active = next_command_active && last_rc_update_time_set && last_position_update_time_set;
	if(!last_rc_update_time_set || (last_rc_update_time_set && ((current_time - last_rc_update_time) > RC_TRIGGER_TIMEOUT))){
		next_command_active = false;
	}
	if(!last_position_update_time_set || (last_position_update_time_set && ((current_time - last_position_update_time) > POSITION_TIMEOUT))){
		next_command_active = false;
	}
	if(!last_attitude_update_time_set || (last_attitude_update_time_set && ((current_time - last_attitude_update_time) > ATTITUDE_TIMEOUT))){
		next_command_active = false;
	}
	if(mode == Mode::OFFBOARD && (!last_trajectory_setpoint_update_time_set || (last_trajectory_setpoint_update_time_set && ((current_time - last_trajectory_setpoint_update_time) > OFFBOARD_TIMEOUT)))){
		next_command_active = false;
	}
	if(prev_command_active != next_command_active){
		if(next_command_active){ 
			PX4_INFO("Command enabled");
			activation_position[0] = vehicle_local_position.x;
			activation_position[1] = vehicle_local_position.y;
			activation_position[2] = vehicle_local_position.z;
			target_position[0] = activation_position[0];
			target_position[1] = activation_position[1];
			target_position[2] = activation_position[2] - target_height; // FRD;
			activation_orientation[0] = vehicle_attitude.q[0];
			activation_orientation[1] = vehicle_attitude.q[1];
			activation_orientation[2] = vehicle_attitude.q[2];
			activation_orientation[3] = vehicle_attitude.q[3];
			target_orientation[0] = activation_orientation[0];
			target_orientation[1] = activation_orientation[1];
			target_orientation[2] = activation_orientation[2];
			target_orientation[3] = activation_orientation[3];
			if constexpr(MAKE_SOME_NOISE){
				tune_control_s tune_control;
				tune_control.tune_id = 1;
				tune_control.volume = 100; //tune_control_s::VOLUME_LEVEL_DEFAULT;
				tune_control.tune_override = true;
				tune_control.frequency = 2000;
				tune_control.duration = 10000;
				_tune_control_pub.publish(tune_control);
			}
			trajectory.reset(current_time);
		}
		else{
			PX4_INFO("Command disabled");
		}
	}

	command_active = next_command_active;

	rl_tools_command_s command;
	command.timestamp = current_time;
	command.timestamp_sample = current_time;
	command.active = command_active;

	switch(mode){
		case Mode::POSITION:
			command.target_position[0] = target_position[0];
			command.target_position[1] = target_position[1];
			command.target_position[2] = target_position[2];
			command.target_linear_velocity[0] = 0;
			command.target_linear_velocity[1] = 0;
			command.target_linear_velocity[2] = 0;
			break;
		case Mode::TRAJECTORY_TRACKING:
			if(!trajectory.initialized){
				trajectory.reset(current_time);
				activation_position[0] = vehicle_local_position.x;
				activation_position[1] = vehicle_local_position.y;
				activation_position[2] = vehicle_local_position.z;
			}
			else{
				trajectory.update(current_time);
			}
			command.target_position[0] = activation_position[0] + trajectory.position[0];
			command.target_position[1] = activation_position[1] + trajectory.position[1];
			command.target_position[2] = activation_position[2] + trajectory.position[2];
			command.target_linear_velocity[0] = trajectory.linear_velocity[0];
			command.target_linear_velocity[1] = trajectory.linear_velocity[1];
			command.target_linear_velocity[2] = trajectory.linear_velocity[2];
			break;
		case Mode::STEP_RESPONSE:
			command.target_position[0] = activation_position[0] + step_response_offset[0];
			command.target_position[1] = activation_position[1] + step_response_offset[1];
			command.target_position[2] = activation_position[2] + step_response_offset[2];
			command.target_linear_velocity[0] = 0;
			command.target_linear_velocity[1] = 0;
			command.target_linear_velocity[2] = 0;
			break;
		case Mode::OFFBOARD:
			if(!last_trajectory_setpoint_update_time_set || (last_trajectory_setpoint_update_time_set && ((current_time - last_trajectory_setpoint_update_time) > OFFBOARD_TIMEOUT))){
				if(!offboard_trajectory_setpoint_missing_error_message_sent){
					PX4_ERR("No trajectory setpoint received, disabling command, you might need to switch to OFFBOARD");
					offboard_trajectory_setpoint_missing_error_message_sent = true;
				}
				command.active = false;
				command.target_position[0] = vehicle_local_position.x;
				command.target_position[1] = vehicle_local_position.y;
				command.target_position[2] = vehicle_local_position.z;
				command.target_linear_velocity[0] = 0;
				command.target_linear_velocity[1] = 0;
				command.target_linear_velocity[2] = 0;
				break;
			}
			else{
				offboard_trajectory_setpoint_missing_error_message_sent = false;
			}
			command.target_position[0] = trajectory_setpoint.position[0];
			command.target_position[1] = trajectory_setpoint.position[1];
			command.target_position[2] = trajectory_setpoint.position[2];
			command.target_linear_velocity[0] = trajectory_setpoint.velocity[0];
			command.target_linear_velocity[1] = trajectory_setpoint.velocity[1];
			command.target_linear_velocity[2] = trajectory_setpoint.velocity[2];
			break;
		default:
			break;
	}
	float w = target_orientation[0];
	// float x = activation_orientation[1];
	// float y = activation_orientation[2];
	float z = target_orientation[3];
	float norm_yaw = sqrt(w*w + z*z);
	float w_yaw, x_yaw, y_yaw, z_yaw;
	if(norm_yaw > 1e-6f){
		// we can isolate the yaw component because it is around the Z axis in the world frame (ZYX convention)
		w_yaw = w / norm_yaw;
		x_yaw = 0;
		y_yaw = 0;
		z_yaw = z / norm_yaw;
	}
	else{
		w_yaw = 1;
		x_yaw = 0;
		y_yaw = 0;
		z_yaw = 0;
		PX4_WARN("Singularity in extracting yaw from attitude");
	}
	command.target_orientation[0] = w_yaw;
	command.target_orientation[1] = x_yaw;
	command.target_orientation[2] = y_yaw;
	command.target_orientation[3] = z_yaw;
	_rl_tools_command_pub.publish(command);
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
	PX4_INFO_RAW("mode: ");
	switch(mode){
		case Mode::POSITION:
			PX4_INFO_RAW("POSITION\n");
			break;
		case Mode::TRAJECTORY_TRACKING:
			PX4_INFO_RAW("TRAJECTORY_TRACKING\n");
			break;
		case Mode::STEP_RESPONSE:
			PX4_INFO_RAW("STEP_RESPONSE\n");
			break;
		default:
			PX4_INFO_RAW("UNKNOWN\n");
			break;	
	}
	PX4_INFO_RAW("activation_position:\n\t%f\n\t%f\n\t%f\n", (double)activation_position[0], (double)activation_position[1], (double)activation_position[2]);
	PX4_INFO_RAW("target_position:\n\t%f\n\t%f\n\t%f\n", (double)target_position[0], (double)target_position[1], (double)target_position[2]);
	PX4_INFO_RAW("activation_orientation:\n\t%f\n\t%f\n\t%f\n\t%f\n", (double)activation_orientation[0], (double)activation_orientation[1], (double)activation_orientation[2], (double)activation_orientation[3]);
	PX4_INFO_RAW("target_orientation:\n\t%f\n\t%f\n\t%f\n\t%f\n", (double)target_orientation[0], (double)target_orientation[1], (double)target_orientation[2], (double)target_orientation[3]);
	PX4_INFO_RAW("command_active: %s\n", command_active ? "true" : "false");
	PX4_INFO_RAW("target_height: %f\n", (double)target_height);
	PX4_INFO_RAW("step_response_offset:\n\t%f\n\t%f\n\t%f\n", (double)step_response_offset[0], (double)step_response_offset[1], (double)step_response_offset[2]);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

void print_custom_command_usage(){
	PX4_INFO_RAW("- overwrite [on/off]\n");
	PX4_INFO_RAW("- set_target_height xx.xx ([m])\n");
	PX4_INFO_RAW("- set_target_position xx.xx yy.yy zz.zz ([m], FRD!)\n");
	PX4_INFO_RAW("- set_target_yaw zz.zz ([rad], FRD!)\n");
	PX4_INFO_RAW("- set_mode {POSITION,TRAJECTORY_TRACKING,STEP_RESPONSE,OFFBOARD}\n");
	PX4_INFO_RAW("- set_trajectory_scale ([m], default figure-eight is 2mx1m\n");
	PX4_INFO_RAW("- set_trajectory_interval ([s], default interval is 5.5s\n");
	PX4_INFO_RAW("- set_step_response_offset xx.xx yy.yy zz.zz ([m], FRD!\n");
}
int RLtoolsCommander::custom_command(int argc, char *argv[])
{
	bool print_usage = true;
	int retval = -1;
	if(argc > 0){
		if(strcmp(argv[0], "overwrite") == 0){
			if(argc > 1){
				if(strcmp(argv[1], "on") == 0){
					get_instance()->overwrite = true;
					PX4_INFO_RAW("overwrite on\n");
					print_usage = false;
					retval = 0;
				}
				else{
					if(strcmp(argv[1], "off") == 0){
						get_instance()->overwrite = false;
						PX4_INFO_RAW("overwrite off\n");
						print_usage = false;
						retval = 0;
					}
				}
			}
		}
		if(strcmp(argv[0], "set_target_height") == 0){
			if(argc > 1){
				float new_target_height = atof(argv[1]);
				PX4_INFO_RAW("Setting target height from %f to %f\n", (double)get_instance()->target_height, (double)new_target_height);
				get_instance()->target_height = new_target_height;
				print_usage = false;
				retval = 0;
			}
		}
		if(strcmp(argv[0], "set_target_position") == 0){
			if(argc > 3){
				float new_target_position[3];
				new_target_position[0] = atof(argv[1]);
				new_target_position[1] = atof(argv[2]);
				new_target_position[2] = atof(argv[3]);
				PX4_INFO_RAW("Setting target position from (%f %f %f) to (%f %f %f)\n",
					(double)get_instance()->target_position[0],
					(double)get_instance()->target_position[1],
					(double)get_instance()->target_position[2],
					(double)new_target_position[0],
					(double)new_target_position[1],
					(double)new_target_position[2]
				);
				get_instance()->target_position[0] = new_target_position[0];
				get_instance()->target_position[1] = new_target_position[1];
				get_instance()->target_position[2] = new_target_position[2];
				print_usage = false;
				retval = 0;
			}
		}
		if(strcmp(argv[0], "set_target_yaw") == 0){
			if(argc > 1){
				float new_target_yaw = atof(argv[1]);
				float old_target_yaw = asinf(get_instance()->target_orientation[3])*2;
				PX4_INFO_RAW("Setting the target yaw from %f (%f degrees) to %f (%f degrees)\n",
					(double)old_target_yaw,
					(double)old_target_yaw*180.0/M_PI,
					(double)new_target_yaw,
					(double)new_target_yaw*180.0/M_PI
				);
				get_instance()->target_orientation[0] = cosf(new_target_yaw/2);
				get_instance()->target_orientation[1] = 0;
				get_instance()->target_orientation[2] = 0;
				get_instance()->target_orientation[3] = sinf(new_target_yaw/2);
				print_usage = false;
				retval = 0;
			}
		}
		if(strcmp(argv[0], "set_trajectory_scale") == 0){
			if(argc > 1){
				float new_scale = atof(argv[1]);
				float old_scale = get_instance()->trajectory.scale;
				PX4_INFO_RAW("Setting the trajectory scale from %f to %f\n", (double)old_scale, (double)new_scale);
				get_instance()->trajectory.scale = new_scale;
				print_usage = false;
				retval = 0;
			}
		}
		if(strcmp(argv[0], "set_trajectory_interval") == 0){
			if(argc > 1){
				float new_interval = atof(argv[1]);
				float old_interval = get_instance()->trajectory.interval;
				PX4_INFO_RAW("Setting the trajectory interval from %f to %f\n", (double)old_interval, (double)new_interval);
				get_instance()->trajectory.interval = new_interval;
				print_usage = false;
				retval = 0;
			}
		}
		if(strcmp(argv[0], "set_step_response_offset") == 0){
			if(argc > 3){
				float new_offset[3];
				new_offset[0] = atof(argv[1]);
				new_offset[1] = atof(argv[2]);
				new_offset[2] = atof(argv[3]);
				PX4_INFO_RAW("Setting the step response offset from (%f %f %f) to (%f %f %f)\n",
					(double)get_instance()->step_response_offset[0],
					(double)get_instance()->step_response_offset[1],
					(double)get_instance()->step_response_offset[2],
					(double)new_offset[0],
					(double)new_offset[1],
					(double)new_offset[2]
				);
				get_instance()->step_response_offset[0] = new_offset[0];
				get_instance()->step_response_offset[1] = new_offset[1];
				get_instance()->step_response_offset[2] = new_offset[2];
				print_usage = false;
				retval = 0;
			}
		}
		if(strcmp(argv[0], "set_mode") == 0){
			if(argc > 1){
				const char* mode_names[] = {
					"POSITION",
					"TRAJECTORY_TRACKING",
					"STEP_RESPONSE",
					"OFFBOARD"
				};
				if(strcmp(argv[1], "POSITION") == 0){
					PX4_INFO_RAW("Setting mode from %s to %s\n", mode_names[(uint8_t)get_instance()->mode], argv[1]);
					get_instance()->mode = Mode::POSITION;
					print_usage = false;
					retval = 0;
				}
				else{
					if(strcmp(argv[1], "TRAJECTORY_TRACKING") == 0){
						PX4_INFO_RAW("Setting mode from %s to %s\n", mode_names[(uint8_t)get_instance()->mode], argv[1]);
						if(get_instance()->mode != Mode::TRAJECTORY_TRACKING){
							PX4_INFO_RAW("Resetting the trajectory\n");
							get_instance()->trajectory.initialized = false;
						}
						get_instance()->mode = Mode::TRAJECTORY_TRACKING;
						print_usage = false;
						retval = 0;
					}
					else{
						if(strcmp(argv[1], "STEP_RESPONSE") == 0){
							PX4_INFO_RAW("Setting mode from %s to %s\n", mode_names[(uint8_t)get_instance()->mode], argv[1]);
							get_instance()->mode = Mode::STEP_RESPONSE;
							print_usage = false;
							retval = 0;
						}
						else{
							if(strcmp(argv[1], "OFFBOARD") == 0){
								PX4_INFO_RAW("Setting mode from %s to %s\n", mode_names[(uint8_t)get_instance()->mode], argv[1]);
								get_instance()->mode = Mode::OFFBOARD;
								print_usage = false;
								retval = 0;
							}
							else{
								PX4_INFO_RAW("Unknown mode %s\n", argv[1]);
							}
						}
					}
				}
			}
		}
	}
	if(print_usage){
		PX4_INFO_RAW("Usage:\n");
		print_custom_command_usage();
	}
	return retval;
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
	PX4_INFO_RAW("Additional commands:\n");
	print_custom_command_usage();

	return 0;
}

extern "C" __EXPORT int rl_tools_commander_main(int argc, char *argv[])
{
	return RLtoolsCommander::main(argc, argv);
}
