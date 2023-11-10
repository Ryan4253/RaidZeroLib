#pragma once

#include<mutex>

#ifndef THREADS_STD
#include "api.h"
#include "pros/apix.h"

#endif

#ifdef THREADS_STD
using CROSSPLATFORM_MUTEX_T = std::mutex;

#else
using CROSSPLATFORM_MUTEX_T = pros::Mutex;

#endif

namespace rz {

class CrossPlatformMutex {
    public:
    CrossPlatformMutex() = default;

    void lock();

    void unlock();

    protected:
    CROSSPLATFORM_MUTEX_T mutex;
};

}; // namespace rz