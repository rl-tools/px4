#include "BackpropToolsBenchmark.hpp"

BackpropToolsBenchmark::BackpropToolsBenchmark() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	lic::malloc(device, buffers);
	lic::malloc(device, output);
}

BackpropToolsBenchmark::~BackpropToolsBenchmark()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	lic::free(device, buffers);
	lic::free(device, output);
}

bool BackpropToolsBenchmark::init()
{
	ScheduleOnInterval(1000000_us); // 2000 us interval, 200 Hz rate
	this->init_time = hrt_absolute_time();

	return true;
}

void BackpropToolsBenchmark::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_count(_loop_interval_perf);

	// DTYPE buffer_tick_memory[mlp_1::SPEC::HIDDEN_DIM];
	// DTYPE buffer_tock_memory[mlp_1::SPEC::HIDDEN_DIM];
	// lic::Matrix<lic::matrix::Specification<DTYPE, TI, 1, mlp_1::SPEC::HIDDEN_DIM, mlp_1::SPEC::MEMORY_LAYOUT>> buffer_tick = {(DTYPE*)buffer_tick_memory};
	// lic::Matrix<lic::matrix::Specification<DTYPE, TI, 1, mlp_1::SPEC::HIDDEN_DIM, mlp_1::SPEC::MEMORY_LAYOUT>> buffer_tock = {(DTYPE*)buffer_tock_memory};



	uint32_t start, end;
	perf_begin(_loop_perf);
	int iterations = 10000;
	auto input_sample = lic::row(device, input::container, 0);
	auto output_sample = lic::row(device, output, 0);
	start = hrt_absolute_time();
	for(int iteration_i = 0; iteration_i < iterations; iteration_i++){
		lic::evaluate(device, mlp_1::mlp, input_sample, output_sample, buffers);
	}
	end = hrt_absolute_time();
	perf_end(_loop_perf);
	PX4_INFO("backprop_tools_benchmark: %dus", (int)(end- start));
	// for(TI batch_i = 0; batch_i < BATCH_SIZE; batch_i++){
	// 	for(TI input_i = 0; input_i < mlp_1::SPEC::INPUT_DIM; input_i++){
	// 		PX4_INFO("input[%d][%d]: %f", batch_i, input_i, get(input::container, batch_i, input_i));
	// 	}
	// }
	for(TI batch_i = 0; batch_i < BATCH_SIZE; batch_i++){
		for(TI output_i = 0; output_i < mlp_1::SPEC::OUTPUT_DIM; output_i++){
			PX4_INFO("output[%d][%d]: %f (diff %f)", batch_i, output_i, get(output, batch_i, output_i), lic::get(output, batch_i, output_i) - lic::get(expected_output::container, batch_i, output_i));
		}
	}
	PX4_INFO("evaluation time: %dus", (int)(end - start));
}

int BackpropToolsBenchmark::task_spawn(int argc, char *argv[])
{
	BackpropToolsBenchmark *instance = new BackpropToolsBenchmark();

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

int BackpropToolsBenchmark::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int BackpropToolsBenchmark::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BackpropToolsBenchmark::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
BackpropTools Benchmark

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("backprop_tools_benchmark", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int backprop_tools_benchmark_main(int argc, char *argv[])
{
	return BackpropToolsBenchmark::main(argc, argv);
}
