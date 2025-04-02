#include "talking/common.hpp"

bool waitForReady(shared_state& state){
    std::unique_lock<std::mutex> lock(state.pose_mtx);
    state.cv.wait(lock, [&state]{
        return state.ready;
    });
    return state.ready;
}