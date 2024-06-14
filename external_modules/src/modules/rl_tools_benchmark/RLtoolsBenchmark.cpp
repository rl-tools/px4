#include "RLtoolsBenchmark.hpp"

RLtoolsBenchmark::RLtoolsBenchmark() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	rlt::malloc(device, buffers);
	rlt::malloc(device, output);

	rng = rlt::random::default_engine(device.random, 0);
}

RLtoolsBenchmark::~RLtoolsBenchmark()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	rlt::free(device, buffers);
	rlt::free(device, output);
}

bool RLtoolsBenchmark::init()
{
	ScheduleOnInterval(1000000_us); // 2000 us interval, 200 Hz rate
	this->init_time = hrt_absolute_time();

	return true;
}

void RLtoolsBenchmark::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_count(_loop_interval_perf);

	// T buffer_tick_memory[mlp_1::SPEC::HIDDEN_DIM];
	// T buffer_tock_memory[mlp_1::SPEC::HIDDEN_DIM];
	// rlt::Matrix<rlt::matrix::Specification<T, TI, 1, mlp_1::SPEC::HIDDEN_DIM, mlp_1::SPEC::MEMORY_LAYOUT>> buffer_tick = {(T*)buffer_tick_memory};
	// rlt::Matrix<rlt::matrix::Specification<T, TI, 1, mlp_1::SPEC::HIDDEN_DIM, mlp_1::SPEC::MEMORY_LAYOUT>> buffer_tock = {(T*)buffer_tock_memory};



	hrt_abstime start, end;
	perf_begin(_loop_perf);
	int iterations = 10000;
	auto input_sample = rlt::row(device, rl_tools_export::input::container, 0);
	auto output_sample = rlt::row(device, output, 0);
	start = hrt_absolute_time();
	for(int iteration_i = 0; iteration_i < iterations; iteration_i++){
		rlt::evaluate(device, rl_tools_export::model::module, input_sample, output_sample, buffers, rng);
	}
	end = hrt_absolute_time();
	perf_end(_loop_perf);
	T inference_frequency = (T)iterations/((T)(end - start)/1e6);
	PX4_INFO("rl_tools_benchmark: %dHz", (int)(inference_frequency));
	// for(TI batch_i = 0; batch_i < BATCH_SIZE; batch_i++){
	// 	for(TI input_i = 0; input_i < mlp_1::SPEC::INPUT_DIM; input_i++){
	// 		PX4_INFO("input[%d][%d]: %f", batch_i, input_i, get(input::container, batch_i, input_i));
	// 	}
	// }
	for(TI batch_i = 0; batch_i < BATCH_SIZE; batch_i++){
		for(TI output_i = 0; output_i < rl_tools_export::model::MODEL::OUTPUT_DIM; output_i++){
			PX4_INFO("output[%d][%d]: %f (diff %f)", batch_i, output_i, get(output, batch_i, output_i), rlt::get(output, batch_i, output_i) - rlt::get(rl_tools_export::output::container, batch_i, output_i));
		}
	}
	PX4_INFO("evaluation time: %dus", (int)(end - start));
}

int RLtoolsBenchmark::task_spawn(int argc, char *argv[])
{
	RLtoolsBenchmark *instance = new RLtoolsBenchmark();

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

int RLtoolsBenchmark::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int RLtoolsBenchmark::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RLtoolsBenchmark::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLtools Benchmark

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rl_tools_benchmark", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rl_tools_benchmark_main(int argc, char *argv[])
{
	return RLtoolsBenchmark::main(argc, argv);
}
