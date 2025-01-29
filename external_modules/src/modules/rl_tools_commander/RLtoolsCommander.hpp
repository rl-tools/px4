#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/rl_tools_command.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/manual_control_setpoint.h>

using namespace time_literals;

class RLtoolsCommander : public ModuleBase<RLtoolsCommander>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RLtoolsCommander();
	~RLtoolsCommander() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	struct FigureEight{
		// parameters
		float warmup_time = 3;
		float interval = 10;
		float scale = 1;
		// state
		bool initialized = false;
		hrt_abstime start_time;
		hrt_abstime last_update_time;
		float progress = 0;
		// output
		float position[3];
		float linear_velocity[3];
		void reset(hrt_abstime timestamp){
			initialized = true;
			start_time = timestamp;
			last_update_time = timestamp;
			progress = 0;
			for(int i = 0; i < 3; i++){
				position[i] = 0;
				linear_velocity[i] = 0;
			}
		}
		void update(hrt_abstime timestamp);
	};
	void Run() override;

	enum class Mode: uint8_t{
		POSITION = 0,
		TRAJECTORY_TRACKING = 1,
		STEP_RESPONSE = 2
	};

	void parameters_update();
	void toggle_integral_gain(bool use_model);
	
	uORB::SubscriptionInterval _parameter_update_sub{ ORB_ID(parameter_update), 1_s };
	uORB::Subscription _manual_control_input_sub{ORB_ID(manual_control_input)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Publication<rl_tools_command_s> _rl_tools_command_pub{ORB_ID(rl_tools_command)};
	uORB::Publication<tune_control_s> _tune_control_pub{ORB_ID(tune_control)};

	static constexpr bool MAKE_SOME_NOISE = true;
	static constexpr bool SCALE_OUTPUT_WITH_THROTTLE = true;
	static constexpr float DEFAULT_TARGET_HEIGHT = 0.0;

	hrt_abstime last_rc_update_time, last_position_update_time, last_attitude_update_time;
	vehicle_local_position_s vehicle_local_position;
	vehicle_attitude_s vehicle_attitude;
	bool last_rc_update_time_set = false, last_position_update_time_set = false, last_attitude_update_time_set = false;
	float activation_position[3] = {0, 0, 0};
	float target_position[3] = {0, 0, 0};
	float activation_orientation[4] = {1, 0, 0, 0};
	float target_orientation[4] = {1, 0, 0, 0};
	bool command_active = false;
	float target_height = DEFAULT_TARGET_HEIGHT;
	bool overwrite = false;
	// Mode mode = POSITION;
	static constexpr Mode DEFAULT_MODE = Mode::POSITION;
	// static constexpr Mode DEFAULT_MODE = Mode::TRAJECTORY_TRACKING;
	Mode mode = DEFAULT_MODE;
	FigureEight trajectory;
	float step_response_offset[3] = {1, 0, 0};

	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// integral gain parameters
	float _xy_vel_i_acc = 0.0f;
	float _z_vel_i_acc = 0.0f;
	float _rollrate_i = 0.0f;
	float _pitchrate_i = 0.0f;
	float _yawrate_i = 0.0f;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>)_param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>)_param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::MC_ROLLRATE_I>)_param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_PITCHRATE_I>)_param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_YAWRATE_I>)_param_mc_yawrate_i
	)
};
