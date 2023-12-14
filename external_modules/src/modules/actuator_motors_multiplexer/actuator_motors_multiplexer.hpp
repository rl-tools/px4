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
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/manual_control_setpoint.h>

using namespace time_literals;

class ActuatorMotorsMultiplexer : public ModuleBase<ActuatorMotorsMultiplexer>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ActuatorMotorsMultiplexer();
	~ActuatorMotorsMultiplexer() override;

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
	uORB::SubscriptionCallbackWorkItem _actuator_motors_sub{this, ORB_ID(actuator_motors)};
	uORB::SubscriptionCallbackWorkItem _actuator_motors_rl_tools_sub{this, ORB_ID(actuator_motors_rl_tools)};
	uORB::Publication<actuator_motors_s> _actuator_motors_mux_pub{ORB_ID(actuator_motors_mux)};

	uint32_t init_time, last_rl_tools_output_time, last_rc_update_time;
	bool last_rl_tools_output_time_set = false, last_rc_update_time_set = false;
	bool use_original_controller = true;

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

};
