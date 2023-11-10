#include "RaidZeroLib/api/Utility/CrossPlatformThread.hpp"

namespace rz {

CrossPlatformThread::CrossPlatformThread(void (*ptr)(void*), void* params, const char* const name)
#ifdef THREADS_STD
    : thread(ptr, params){}
#else
    : thread(pros::c::task_create(ptr, params, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, name)) {
}
#endif

      CrossPlatformThread::~CrossPlatformThread() {
#ifdef THREADS_STD
    thread.join();

#else
    if (thread.get_state() != pros::E_TASK_STATE_DELETED) {
        thread.remove();
    }

#endif
}

std::uint32_t CrossPlatformThread::notifyTake(const bool clearOnExit, const std::uint32_t timeout) {
#ifdef THREADS_STD
    return 0;

#else
    return pros::c::task_notify_take(clearOnExit, timeout);

#endif
}

std::uint32_t CrossPlatformThread::notify() {
#ifdef THREADS_STD
    return 1;

#else
    return thread.notify();

#endif
}

std::string CrossPlatformThread::getName() {
#ifdef THREADS_STD
    std::ostringstream ss;
    ss << std::this_thread::get_id();
    return ss.str();

#else
    return thread.get_name();

#endif
}

} // namespace rz