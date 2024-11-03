#include "ControllerTask.h"

using namespace imsl::vehiclecontrol;

void call_pose_control_task(void *arg) {
  auto *controllerTask = static_cast<ControllerTask<real_t> *>(arg);
  controllerTask->PoseControlTask();
}

void call_wheel_control_task(void *arg) {
  auto *controllerTask = static_cast<ControllerTask<real_t> *>(arg);
  controllerTask->WheelControlTask();
}
