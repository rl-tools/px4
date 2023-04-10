#pragma once

#include <layer_in_c/operations/arm.h>
#include <layer_in_c/nn/layers/dense/operations_arm.h>
#include <layer_in_c/nn_models/mlp/operations_generic.h>
#include <test_layer_in_c_nn_models_mlp_persist_code.h>
#include <test_layer_in_c_nn_models_mlp_evaluation.h>

namespace lic = layer_in_c;


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

class BackpropToolsBenchmark : public ModuleBase<BackpropToolsBenchmark>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BackpropToolsBenchmark();
	~BackpropToolsBenchmark() override;

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

	using DEVICE = layer_in_c::devices::DefaultARM;
	DEVICE device;
	using TI = typename mlp_1::SPEC::TI;
	using DTYPE = typename mlp_1::SPEC::T;
	static constexpr TI BATCH_SIZE = decltype(input::matrix)::ROWS;

	uint32_t init_time;
	decltype(mlp_1::mlp)::template Buffers<BATCH_SIZE> buffers;// = {buffer_tick, buffer_tock};
	lic::Matrix<lic::matrix::Specification<DTYPE, TI, BATCH_SIZE, mlp_1::SPEC::OUTPUT_DIM, lic::matrix::layouts::RowMajorAlignment<TI, 1>>> output;
};
