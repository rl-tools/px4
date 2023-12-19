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
#include <uORB/topics/vehicle_local_position.h>
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
	void Run() override;
	uORB::Subscription _manual_control_input_sub{ORB_ID(manual_control_input)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Publication<rl_tools_command_s> _rl_tools_command_pub{ORB_ID(rl_tools_command)};
	uORB::Publication<tune_control_s> _tune_control_pub{ORB_ID(tune_control)};

	static constexpr bool MAKE_SOME_NOISE = true;
	static constexpr bool SCALE_OUTPUT_WITH_THROTTLE = true;
	static constexpr float TARGET_HEIGHT = 0.0;
	// static constexpr float TARGET_HEIGHT = 0.2;

	uint32_t last_rc_update_time, last_position_update_time;
	vehicle_local_position_s vehicle_local_position;
	bool last_rc_update_time_set = false, last_position_update_time_set = false;
	float activation_position[3] = {0, 0, 0};
	bool command_active = false;

	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
