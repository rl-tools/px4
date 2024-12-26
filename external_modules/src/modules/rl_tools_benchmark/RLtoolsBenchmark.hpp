#pragma once

#define RL_TOOLS_NN_DISABLE_GENERIC_FORWARD_BACKWARD
#include <rl_tools/operations/arm.h>
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
#include <rl_tools/nn/layers/dense/operations_arm/dsp.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include "model.h"

namespace rlt = rl_tools;


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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class RLtoolsBenchmark : public ModuleBase<RLtoolsBenchmark>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RLtoolsBenchmark();
	~RLtoolsBenchmark() override;

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

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	using DEV_SPEC = rl_tools::devices::DefaultARMSpecification;
	// using DEVICE = rl_tools::devices::arm::DSP<DEV_SPEC>;
	using DEVICE = rl_tools::devices::arm::OPT<DEV_SPEC>;
	DEVICE device;
	decltype(rlt::random::default_engine(device.random)) rng;
	using TI = typename rl_tools::checkpoint::actor::TYPE::SPEC::TI;
	using T = typename rl_tools::checkpoint::actor::TYPE::SPEC::T;
	static_assert(rl_tools::checkpoint::example::input::SHAPE::LENGTH == 3);
	static_assert(rl_tools::checkpoint::example::input::SHAPE::template GET<0> == 1, "Sequential Models not supported, yet");
	static constexpr TI BATCH_SIZE = rl_tools::checkpoint::example::input::SHAPE::template GET<1>;

	// static_assert(BATCH_SIZE == 1);
	hrt_abstime init_time;
	rl_tools::checkpoint::actor::TYPE::template Buffer<> buffers;
	rlt::Tensor<rlt::tensor::Specification<T, TI, rl_tools::checkpoint::example::output::SHAPE>> output; 
};
