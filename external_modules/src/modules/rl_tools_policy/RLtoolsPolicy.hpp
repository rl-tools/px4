#pragma once

#include <rl_tools/inference/applications/l2f/c_interface.h>

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
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/rl_tools_command.h>
#include <uORB/topics/rl_tools_policy_status.h>
#include <uORB/topics/rl_tools_policy_input.h>
#include <uORB/topics/tune_control.h>



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
	using TI = size_t;
	using T = float;
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
	static constexpr TI OBSERVATION_TIMEOUT_LOCAL_POSITION = 100 * 1000;
	static constexpr TI OBSERVATION_TIMEOUT_VISUAL_ODOMETRY = 100 * 1000;
	static constexpr TI OBSERVATION_TIMEOUT_ATTITUDE = 50 * 1000;
	static constexpr TI COMMAND_TIMEOUT = 100 * 1000;
	static constexpr T RESET_PREVIOUS_ACTION_VALUE = 0; // -1 to 1

	enum class OdometrySource: TI{
		LOCAL_POSITION = 0,
		VISUAL_ODOMETRY = 1,
	};
	static constexpr OdometrySource ODOMETRY_SOURCE = OdometrySource::VISUAL_ODOMETRY;

	T min(T a, T b) {
		return a < b ? a : b;
	}

	T max_position_error = 0.5; // min(rl_tools::checkpoint::meta::environment::parameters::mdp::init::max_position, rl_tools::checkpoint::meta::environment::parameters::mdp::termination::position_threshold);
	T max_velocity_error = 1.0; // min(rl_tools::checkpoint::meta::environment::parameters::mdp::init::max_linear_velocity, rl_tools::checkpoint::meta::environment::parameters::mdp::termination::linear_velocity_threshold);

	void Run() override;


	// node state
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_odometry_s _vehicle_visual_odometry{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	vehicle_attitude_s _vehicle_attitude{};
	rl_tools_command_s _rl_tools_command{};
	hrt_abstime timestamp_last_local_position, timestamp_last_visual_odometry, timestamp_last_visual_odometry_stale, timestamp_last_angular_velocity, timestamp_last_attitude, timestamp_last_command, timestamp_last_manual_control_input;
	bool timestamp_last_local_position_set = false, timestamp_last_visual_odometry_set = false, timestamp_last_visual_odometry_stale_set = false, timestamp_last_angular_velocity_set = false, timestamp_last_attitude_set = false, timestamp_last_command_set = false, timestamp_last_manual_control_input_set = false;
	bool timeout_message_sent = false;
	bool previous_command_stale = false;
	bool previous_active = false;

	TI visual_odometry_stale_counter = 0;


	T position[3];
	T linear_velocity[3];
	
	uORB::Subscription _rl_tools_command_sub{ORB_ID(rl_tools_command)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionCallbackWorkItem _vehicle_visual_odometry_sub{this, ORB_ID(vehicle_visual_odometry)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Publication<actuator_motors_s> _actuator_motors_rl_tools_pub{ORB_ID(actuator_motors_rl_tools)};
	uORB::Publication<rl_tools_policy_status_s> _rl_tools_policy_status_pub{ORB_ID(rl_tools_policy_status)};
	uORB::Publication<rl_tools_policy_input_s> _rl_tools_policy_input_pub{ORB_ID(rl_tools_policy_input)};
	uORB::Publication<tune_control_s> _tune_control_pub{ORB_ID(tune_control)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t	_loop_interval_policy_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval_policy")};

	// using DEV_SPEC = rlt::devices::DefaultARMSpecification;
	// // using DEVICE = rl_tools::devices::arm::DSP<DEV_SPEC>;
	// using DEVICE = rlt::devices::arm::OPT<DEV_SPEC>;
	// DEVICE device;
	// DEVICE::SPEC::RANDOM::ENGINE<> rng;
	// using ACTOR_TYPE_ORIGINAL = rlt::checkpoint::actor::TYPE;
	// static constexpr TI BATCH_SIZE = 1;
	// using ACTOR_TYPE = ACTOR_TYPE_ORIGINAL::template CHANGE_BATCH_SIZE<TI, BATCH_SIZE>;

	void reset();
	void observe(RLtoolsInferenceApplicationsL2FObservation& observation, TestObservationMode mode);

	static constexpr bool REMAP_FROM_CRAZYFLIE = true; // Policy (Crazyflie assignment) => Quadrotor (PX4 Quadrotor X assignment) PX4 SIH assumes the Quadrotor X configuration, which assumes different rotor positions than the crazyflie mapping (from crazyflie outputs to PX4): 1=>1, 2=>4, 3=>2, 4=>3 
	// controller state

	// messaging state
	static constexpr TI POLICY_INTERVAL_WARNING_THRESHOLD = 100; // us
	static constexpr TI POLICY_FREQUENCY_CHECK_INTERVAL = 1000 * 1000; // 1s
	static constexpr TI POLICY_FREQUENCY_INFO_INTERVAL = 10; // 10 x POLICY_FREQUENCY_CHECK_INTERVAL = 10x

	TI num_statii;
	TI num_healthy_executor_statii_intermediate, num_non_healthy_executor_statii_intermediate, num_healthy_executor_statii_native, num_non_healthy_executor_statii_native;
	RLtoolsInferenceExecutorStatus last_intermediate_status, last_native_status;
	bool last_intermediate_status_set, last_native_status_set;

	TI policy_frequency_check_counter;
	hrt_abstime timestamp_last_policy_frequency_check;
	bool timestamp_last_policy_frequency_check_set = false;

	float previous_action[RL_TOOLS_INTERFACE_APPLICATIONS_L2F_ACTION_DIM];
};
