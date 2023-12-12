message("REAL cmake")

if(NOT DEFINED RL_TOOLS_ROOT)
  message(FATAL_ERROR "RL_TOOLS_ROOT is not defined")
endif()

include(px4_add_library)

px4_add_module(
	MODULE modules__rl_tools_policy
	MAIN rl_tools_policy
	STACK_MAIN 2000
	COMPILE_FLAGS
		-DARM_MATH_LOOPUNROLL
	SRCS
		RLtoolsPolicy.cpp
	# DEPENDS
	# 	px4_work_queue
	EXTERNAL
	)
target_include_directories(
	modules__rl_tools_policy
	PRIVATE
	${RL_TOOLS_ROOT}/include
	${RL_TOOLS_ROOT}/data
)

target_compile_features(modules__rl_tools_policy PRIVATE cxx_std_17)
target_compile_options(modules__rl_tools_policy PRIVATE -Wno-error -DARM_MATH_LOOPUNROLL)




