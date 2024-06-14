#pragma once

#define RL_TOOLS_NAMESPACE_WRAPPER rl_tools_policy
#include <rl_tools/operations/arm.h>
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
// #include <rl_tools/nn/layers/dense/operations_arm/dsp.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include "model.h"

namespace rlt = RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools;


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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/rl_tools_command.h>
#include <uORB/topics/rl_tools_policy_status.h>
#include <uORB/topics/rl_tools_policy_input.h>

using namespace time_literals;

class RLtoolsPolicy : public ModuleBase<RLtoolsPolicy>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RLtoolsPolicy();
	~RLtoolsPolicy() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	using TI = typename rl_tools::checkpoint::actor::MODEL::CONTENT::SPEC::TI;
	using T = typename rl_tools::checkpoint::actor::MODEL::CONTENT::SPEC::T;
	enum class TestObservationMode: TI{
		ANGULAR_VELOCITY = 0,
		ORIENTATION = 1,
		LINEAR_VELOCITY = 2,
		POSITION = 3,
		ACTION_HISTORY = 4,
	};
	// static constexpr TestObservationMode TEST_OBSERVATION_MODE = TestObservationMode::ANGULAR_VELOCITY;
	static constexpr TestObservationMode TEST_OBSERVATION_MODE = TestObservationMode::ACTION_HISTORY;
	hrt_abstime init_time;
	// node constants
	static constexpr TI OBSERVATION_TIMEOUT_ANGULAR_VELOCITY = 10 * 1000;
	static constexpr TI OBSERVATION_TIMEOUT_POSITION = 100 * 1000;
	static constexpr TI OBSERVATION_TIMEOUT_ATTITUDE = 50 * 1000;
	static constexpr TI COMMAND_TIMEOUT = 100 * 1000;

	static constexpr T MAX_POSITION_ERROR = 2.0;
	static constexpr T MAX_VELOCITY_ERROR = 1.0;

	void Run() override;


	// node state
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	vehicle_attitude_s _vehicle_attitude{};
	rl_tools_command_s _rl_tools_command{};
	hrt_abstime timestamp_last_local_position, timestamp_last_angular_velocity, timestamp_last_attitude, timestamp_last_command, timestamp_last_manual_control_input;
	bool timestamp_last_local_position_set = false, timestamp_last_angular_velocity_set = false, timestamp_last_attitude_set = false, timestamp_last_command_set = false, timestamp_last_manual_control_input_set = false;
	bool timeout_message_sent = false;
	bool previous_command_stale = false;
	bool previous_active = false;

	
	uORB::Subscription _rl_tools_command_sub{ORB_ID(rl_tools_command)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Publication<actuator_motors_s> _actuator_motors_rl_tools_pub{ORB_ID(actuator_motors_rl_tools)};
	uORB::Publication<rl_tools_policy_status_s> _rl_tools_policy_status_pub{ORB_ID(rl_tools_policy_status)};
	uORB::Publication<rl_tools_policy_input_s> _rl_tools_policy_input_pub{ORB_ID(rl_tools_policy_input)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t	_loop_interval_policy_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval_policy")};

	using DEV_SPEC = rlt::devices::DefaultARMSpecification;
	// using DEVICE = rl_tools::devices::arm::DSP<DEV_SPEC>;
	using DEVICE = rlt::devices::arm::OPT<DEV_SPEC>;
	DEVICE device;
	decltype(rlt::random::default_engine(device.random)) rng;
	static constexpr TI BATCH_SIZE = decltype(rl_tools::checkpoint::observation::container)::ROWS;

	template <typename OBS_SPEC>
	void observe_rotation_matrix(rlt::Matrix<OBS_SPEC>& observation, TestObservationMode mode);
	void rl_tools_control(TI substep, TestObservationMode mode);
	void clear_action_history();

	static_assert(BATCH_SIZE == 1);
	static constexpr TI TRAINING_CONTROL_INTERVAL = 10000; // us
	static constexpr TI CONTROL_MULTIPLE = 4; 
	static constexpr TI CONTROL_INTERVAL = TRAINING_CONTROL_INTERVAL / CONTROL_MULTIPLE; // 500Hz
	rl_tools::checkpoint::actor::MODEL::template Buffer<BATCH_SIZE> buffers;// = {buffer_tick, buffer_tock};
	static constexpr TI ACTION_HISTORY_LENGTH = 32;
	static constexpr TI ACTION_DIM = 4;
	static constexpr TI EXPECTED_INPUT_DIM = 3 + 9 + 3 + 3 + ACTION_HISTORY_LENGTH * ACTION_DIM;
	static_assert(EXPECTED_INPUT_DIM == rl_tools::checkpoint::actor::MODEL::INPUT_DIM);
	static_assert(rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM == 4);
	// controller buffers 
	rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, BATCH_SIZE, rl_tools::checkpoint::actor::MODEL::INPUT_DIM, rlt::matrix::layouts::RowMajorAlignment<TI, 1>>> input;
	rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, BATCH_SIZE, rl_tools::checkpoint::actor::MODEL::OUTPUT_DIM, rlt::matrix::layouts::RowMajorAlignment<TI, 1>>> output;
	// controller state
	hrt_abstime timestamp_last_forward_pass;
	bool timestamp_last_forward_pass_set = false;
	TI controller_tick = 0;
	TI controller_tick_substep_offset = 0;
	T action_history[ACTION_HISTORY_LENGTH][ACTION_DIM];

	// messaging state
	static constexpr TI POLICY_INTERVAL_WARNING_THRESHOLD = 100; // us
	static constexpr TI POLICY_FREQUENCY_CHECK_INTERVAL = 1000 * 1000; // 1s
	hrt_abstime timestamp_last_policy_frequency_check;
	bool timestamp_last_policy_frequency_check_set = false;
};
