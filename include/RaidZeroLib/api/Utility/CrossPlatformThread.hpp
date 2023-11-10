#pragma once

#ifdef THREADS_STD
#include <sstream>
#include <thread>

#else
#include "api.h"
#include "pros/apix.h"

#endif

namespace rz {

#ifdef THREADS_STD
using CROSSPLATFORM_THREAD_T = std::thread;

#else
using CROSSPLATFORM_THREAD_T = pros::Task;

#endif

class CrossPlatformThread {
    public:
    CrossPlatformThread(void (*ptr)(void*), void* params, const char* const name = "CrossPlatformThread");

    ~CrossPlatformThread();

    static std::uint32_t notifyTake(const bool clearOnExit, const std::uint32_t timeout);

    std::uint32_t notify();

    std::string getName();

    protected:
    CROSSPLATFORM_THREAD_T thread;
};

} // namespace rz