include(px4_add_library)

px4_add_module(
	MODULE modules__rl_tools_commander
	MAIN rl_tools_commander
	STACK_MAIN 2000
	SRCS
		RLtoolsCommander.cpp
	EXTERNAL
	)

target_compile_features(modules__rl_tools_commander PRIVATE cxx_std_17)
