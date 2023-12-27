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
#include <uORB/topics/rl_tools_multiplexer_status.h>

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
	enum class Mode: int{
		SWITCH_BACK = 0,
		TURN_OFF = 1,
		TURN_OFF_AFTER_TIMEOUT = 2
	};
	// static constexpr Mode MODE = Mode::TURN_OFF_AFTER_TIMEOUT;
	static constexpr Mode MODE = Mode::TURN_OFF;
	static constexpr uint64_t SWITCH_BACK_TIMEOUT = 5 * 1000 * 1000;
	static constexpr bool SCALE_OUTPUT_WITH_THROTTLE = false;
	void Run() override;
	static constexpr bool DEACTIVATE_OUTPUTS_AFTER_ACTIVATION = true;
	uORB::Subscription _manual_control_input_sub{ORB_ID(manual_control_input)};
	uORB::SubscriptionCallbackWorkItem _actuator_motors_sub{this, ORB_ID(actuator_motors)};
	uORB::SubscriptionCallbackWorkItem _actuator_motors_rl_tools_sub{this, ORB_ID(actuator_motors_rl_tools)};
	uORB::Publication<actuator_motors_s> _actuator_motors_mux_pub{ORB_ID(actuator_motors_mux)};
	uORB::Publication<rl_tools_multiplexer_status_s> _rl_tools_multiplexer_status_pub{ORB_ID(rl_tools_multiplexer_status)};

	manual_control_setpoint_s manual_control_input;

	uint32_t init_time, last_rl_tools_output_time, last_rc_update_time, last_activation_time;
	bool last_rl_tools_output_time_set = false, last_rc_update_time_set = false, last_activation_time_set = false;
	bool use_original_controller = true;
	bool deactivated = false;
	bool overwrite = false;

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

};
