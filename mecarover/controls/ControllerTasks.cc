#include "ControllerTasks.h"

using namespace imsl::vehiclecontrol;

void PoseControllerTaskFunction(void *arg)
{
	ControllerTasksInterfaces<real_t>* controllerTasks = (ControllerTasksInterfaces<real_t>*) arg;
	log_message(log_info, "pose controller");
	controllerTasks->PoseControlTask(); 
}

void WheelControllerTaskFunction(void *arg)
{
	ControllerTasksInterfaces<real_t>* controllerTasks = (ControllerTasksInterfaces<real_t>*) arg;
	log_message(log_info, "wheel controller");
	controllerTasks->WheelControlTask();
}

// ref pose in automatic mode, TODO implement
int get_ref_pose(PoseV_t *p)
{
	return 0;
}
