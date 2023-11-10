#include "RaidZeroLib/api/Utility/CrossPlatformMutex.hpp"

namespace rz {

void CrossPlatformMutex::lock() {
#ifdef THREADS_STD
    mutex.lock();

#else
    while (!mutex.take(1)) {
    }

#endif
}

void CrossPlatformMutex::unlock() {
#ifdef THREADS_STD
    mutex.unlock();

#else
    mutex.give();

#endif
}

} // namespace rz
