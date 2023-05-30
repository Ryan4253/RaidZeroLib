#include "lib4253/Utility/TaskWrapper.hpp"
namespace lib4253{

void TaskWrapper::loop(){
    throw "task loop isn't overridden!";
}

void TaskWrapper::startTask(const char*  iname){
    task = std::move(std::make_unique<pros::Task>(trampoline, this, iname));
}

void TaskWrapper::pauseTask(){
    task->suspend();
}

void TaskWrapper::resumeTask(){
    task->resume();
}

void TaskWrapper::stopTask(){
    task->remove();
}

char const* TaskWrapper::getName(){
    return task->get_name();
}

void TaskWrapper::trampoline(void* iparam){
    if(iparam){
        TaskWrapper* that = static_cast<TaskWrapper*>(iparam);
        that->loop();
    }
}
}

