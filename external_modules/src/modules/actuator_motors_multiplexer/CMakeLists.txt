include(px4_add_library)

px4_add_module(
	MODULE modules__actuator_motors_multiplexer
	MAIN actuator_motors_multiplexer
	STACK_MAIN 2000
	SRCS
		actuator_motors_multiplexer.cpp
	MODULE_CONFIG
		module.yaml
	EXTERNAL
	)

target_compile_features(modules__actuator_motors_multiplexer PRIVATE cxx_std_17)