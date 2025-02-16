#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <task.h>
#include <ulog.h>

#include <application/hal/hal.hpp>

#include "four_wheel_data.hpp"
#include "shared.hpp"

static TaskHandle_t task_handle;

using namespace robot_params;

using freertos::FourWheelData;

extern "C" {

static void task_impl(void*) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(WHEEL_CTRL_PERIOD_MS.count());

  while (true) {
    auto enc_delta = FourWheelData(hal_encoder_delta_rad());

    xQueueSend(freertos::enc_delta_wheel_ctrl_queue, &enc_delta, 0);
    xQueueOverwrite(freertos::enc_delta_odom_queue, &enc_delta);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
}

namespace freertos {

void task_get_enc_delta_init() {
  constexpr size_t STACK_SIZE = configMINIMAL_STACK_SIZE * 4;
#ifdef FREERTOS_STATIC_INIT
  static StackType_t taskStack[STACK_SIZE];
  static StaticTask_t taskBuffer;
  configASSERT((task_handle = xTaskCreateStatic(
                    task_impl, "get_enc_delta", STACK_SIZE, NULL,
                    osPriorityNormal, taskStack, &taskBuffer)) != NULL);
#else
  configASSERT(xTaskCreate(task_impl, "get_enc_delta", STACK_SIZE, NULL,
                           osPriorityNormal, &task_handle) == pdPASS);
#endif
}
}  // namespace freertos
