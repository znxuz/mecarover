#include "setup.hpp"

#include "recv_vel_task.hpp"

namespace freertos {
void setup() { recv_vel_task_create(); }
}  // namespace freertos
