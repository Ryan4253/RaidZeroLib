#include "main.h"
namespace lib4253{

class TaskWrapper {
protected:
  TaskWrapper() = default;
  TaskWrapper(const TaskWrapper& itask) = delete;
  TaskWrapper(TaskWrapper&& itask) = default;
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
  virtual void startTask(const std::string& iname = "TaskWrapper");

  /**
   * Kill the task.
   */
  virtual void stopTask();

  /**
   * Get the task name.
   *
   * @return The name.
   */
  virtual std::string getName();

private:
  static void trampoline(void* iparam);
  std::unique_ptr<pros::Task> task {nullptr};
};
}