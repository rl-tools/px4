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
#include <uORB/topics/actuator_motors.h>

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
	void Run() override;
	uORB::Publication<actuator_motors_s> _actuator_motors_rl_tools_pub{ORB_ID(actuator_motors_rl_tools)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	using DEV_SPEC = rlt::devices::DefaultARMSpecification;
	// using DEVICE = rl_tools::devices::arm::DSP<DEV_SPEC>;
	using DEVICE = rlt::devices::arm::OPT<DEV_SPEC>;
	DEVICE device;
	using TI = typename rl_tools_export::model::MODEL::CONTENT::SPEC::TI;
	using T = typename rl_tools_export::model::MODEL::CONTENT::SPEC::T;
	static constexpr TI BATCH_SIZE = decltype(rl_tools_export::input::container)::ROWS;

	static_assert(BATCH_SIZE == 1);
	uint32_t init_time;
	rl_tools_export::model::MODEL::template DoubleBuffer<BATCH_SIZE> buffers;// = {buffer_tick, buffer_tock};
	rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, BATCH_SIZE, rl_tools_export::model::MODEL::OUTPUT_DIM, rlt::matrix::layouts::RowMajorAlignment<TI, 1>>> output;
};
