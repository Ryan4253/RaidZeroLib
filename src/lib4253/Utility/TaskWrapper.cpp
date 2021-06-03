#include "main.h"
namespace lib4253{

void TaskWrapper::loop(){
    throw "task loop isn't overridden!";
}

void TaskWrapper::startTask(const std::string& iname = "TaskWrapper"){
    task = std::move(std::make_unique<pros::Task>(trampoline, this, iname));
}

void TaskWrapper::stopTask(){
    task->remove();
}

std::string TaskWrapper::getName(){
    return task->get_name();
}

void TaskWrapper::trampoline(void* iparam){
    if(iparam){
        TaskWrapper* that = static_cast<TaskWrapper*>(iparam);
        that->loop();
    }
}
}

