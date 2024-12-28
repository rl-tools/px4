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
	hrt_abstime start, end;
	perf_begin(_loop_perf);
	int iterations = 1000;
	start = hrt_absolute_time();
	rlt::Mode<rlt::mode::Evaluation<>> mode;
	for(int iteration_i = 0; iteration_i < iterations; iteration_i++){
		rlt::evaluate(device, rl_tools::checkpoint::actor::module, rl_tools::checkpoint::example::input::container, output, buffers, rng, mode);
	}
	end = hrt_absolute_time();
	perf_end(_loop_perf);
	T inference_frequency = (T)iterations/((T)(end - start)/1e6);
	PX4_INFO("rl_tools_benchmark: %dHz", (int)(inference_frequency));
	for(TI batch_i = 0; batch_i < rl_tools::checkpoint::example::output::SHAPE::GET<1>; batch_i++){
		for(TI output_i = 0; output_i < rl_tools::checkpoint::example::output::SHAPE::GET<2>; output_i++){
			PX4_INFO("output[%d][%d]: %f (diff %f)", batch_i, output_i, get(device, output, 0, batch_i, output_i), rlt::get(device, output, 0, batch_i, output_i) - rlt::get(device, rl_tools::checkpoint::example::output::container, 0, batch_i, output_i));
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
