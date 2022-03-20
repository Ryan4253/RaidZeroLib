#pragma once
#include "pros/rtos.hpp"
#include <memory>

namespace lib4253{

class TaskWrapper {
protected:
    TaskWrapper() = default;
    TaskWrapper(const TaskWrapper& itask) = delete;
    //TaskWrapper(TaskWrapper&& itask) = default;
    virtual ~TaskWrapper() = default;
    
    /**
     * Override this function to implement a custom task loop.
     * Will throw if not overridden.
     */
    virtual void loop();

    public:
    /**   
     * Start the task.
     *
     * @param iname The task name, optional.
     */
    virtual void startTask(const char*  iname = "TaskWrapper");

    virtual void resumeTask();

    virtual void pauseTask();

    /**
     * Kill the task.
     */
    virtual void stopTask();

    /**
     * Get the task name.
     *
     * @return The name.
     */
    virtual char const* getName();

    private:
    static void trampoline(void* iparam);
    std::unique_ptr<pros::Task> task {nullptr};
};
}