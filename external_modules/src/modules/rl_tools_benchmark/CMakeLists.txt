# add_subdirectory(CMSIS-DSP)
message("REAL cmake")

if(NOT DEFINED RL_TOOLS_ROOT)
  message(FATAL_ERROR "RL_TOOLS_ROOT is not defined")
endif()

set(CMSIS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5)
set(CMSIS_DSP ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP)

include(px4_add_library)

px4_add_library(modules__rl_tools_benchmark_lib
	${CMSIS_DSP}/Source/MatrixFunctions/MatrixFunctions.c
	# ${CMSIS_DSP}/Source/BasicMathFunctions/BasicMathFunctions.c # conflict with some other rogue module
	${CMSIS_DSP}/Source/BasicMathFunctions/arm_add_f32.c
)
target_include_directories(
	modules__rl_tools_benchmark_lib
	PUBLIC
	${CMSIS_ROOT}/CMSIS/Core/Include
	${CMSIS_DSP}/Include
)
target_compile_options(modules__rl_tools_benchmark_lib PRIVATE -Wno-error)


px4_add_module(
	MODULE modules__rl_tools_benchmark
	MAIN rl_tools_benchmark
	STACK_MAIN 2000
	COMPILE_FLAGS
		-DARM_MATH_LOOPUNROLL
	SRCS
		RLtoolsBenchmark.cpp
	# DEPENDS
	# 	px4_work_queue
	EXTERNAL
	)
target_include_directories(
	modules__rl_tools_benchmark
	PRIVATE
	${RL_TOOLS_ROOT}/include
	${RL_TOOLS_ROOT}/data
)
target_link_libraries(
	modules__rl_tools_benchmark
	PRIVATE
	modules__rl_tools_benchmark_lib
)


target_compile_features(modules__rl_tools_benchmark PRIVATE cxx_std_17)
target_compile_options(modules__rl_tools_benchmark PRIVATE -Wno-error -DARM_MATH_LOOPUNROLL)




