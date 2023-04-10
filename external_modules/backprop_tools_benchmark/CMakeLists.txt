# add_subdirectory(CMSIS-DSP)

set(CMSIS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5)
set(CMSIS_DSP ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP)


px4_add_library(examples__backprop_tools_benchmark_lib
	${CMSIS_DSP}/Source/MatrixFunctions/MatrixFunctions.c
	# ${CMSIS_DSP}/Source/BasicMathFunctions/BasicMathFunctions.c # conflict with some other rogue module
	${CMSIS_DSP}/Source/BasicMathFunctions/arm_add_f32.c
)
target_include_directories(
	examples__backprop_tools_benchmark_lib
	PUBLIC
	${CMSIS_ROOT}/CMSIS/Core/Include
	${CMSIS_DSP}/Include
)
target_compile_options(examples__backprop_tools_benchmark_lib PRIVATE -Wno-error)

px4_add_module(
	MODULE examples__backprop_tools_benchmark
	MAIN backprop_tools_benchmark
	STACK_MAIN 2000
	COMPILE_FLAGS
		-DARM_MATH_LOOPUNROLL
	SRCS
		BackpropToolsBenchmark.cpp
	DEPENDS
		px4_work_queue
	)
target_include_directories(
	examples__backprop_tools_benchmark
	PRIVATE
	/home/jonas/phd/projects/rl_for_control/layer-in-c/include
	/home/jonas/phd/projects/rl_for_control/layer-in-c/data
)
target_link_libraries(
	examples__backprop_tools_benchmark
	PRIVATE
	examples__backprop_tools_benchmark_lib
)


target_compile_features(examples__backprop_tools_benchmark PRIVATE cxx_std_17)
target_compile_options(examples__backprop_tools_benchmark PRIVATE -Wno-error -DARM_MATH_LOOPUNROLL)




