#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/layers/dense/operations_cpu.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include "model.h"
#include <iostream>
namespace rlt = rl_tools;

using DEVICE = rlt::devices::DefaultCPU;
using TI = DEVICE::index_t;
using T = float;
static constexpr TI ACTION_HISTORY_LENGTH = 32;
static constexpr TI EXPECTED_INPUT_DIM = 3 + 9 + 3 + 3 + ACTION_HISTORY_LENGTH * 4;
static constexpr TI OUTPUT_DIM = decltype(rl_tools::checkpoint::actor::model)::OUTPUT_DIM;

int main(){
    DEVICE device;
    rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, 1, EXPECTED_INPUT_DIM>> input;
    rlt::MatrixDynamic<rlt::matrix::Specification<T, TI, 1, OUTPUT_DIM>> output;
    decltype(rl_tools::checkpoint::actor::model)::template Buffer<1> buffer;

    rlt::malloc(device, input);
    rlt::malloc(device, output);
    rlt::malloc(device, buffer);

    rlt::set(input, 0,  0+0, 0);
    rlt::set(input, 0,  0+1, 0);
    rlt::set(input, 0,  0+2, 0);
    rlt::set(input, 0,  3+0, 1);
    rlt::set(input, 0,  3+1, 0);
    rlt::set(input, 0,  3+2, 0);
    rlt::set(input, 0,  6+0, 0);
    rlt::set(input, 0,  6+1, 1);
    rlt::set(input, 0,  6+2, 0);
    rlt::set(input, 0,  9+0, 0);
    rlt::set(input, 0,  9+1, 0);
    rlt::set(input, 0,  9+2, 1);
    rlt::set(input, 0, 12+0, 0);
    rlt::set(input, 0, 12+1, 0);
    rlt::set(input, 0, 12+2, 0);
    rlt::set(input, 0, 15+0, 1);
    rlt::set(input, 0, 15+1, 0);
    rlt::set(input, 0, 15+2, 0);


    T default_action_history = rl_tools::checkpoint::meta::action_history_init;

    for(TI step_i=0; step_i < ACTION_HISTORY_LENGTH; step_i++){
        for(TI action_i=0; action_i < OUTPUT_DIM; action_i++){
            rlt::set(input, 0, 18 + step_i * OUTPUT_DIM + action_i, default_action_history);
        }
    }

	rlt::evaluate(device, rl_tools::checkpoint::actor::model, input, output, buffer);

    for(TI action_i=0; action_i < OUTPUT_DIM; action_i++){
        std::cout << rlt::get(output, 0, action_i) << std::endl;
    }
    return 0;
}