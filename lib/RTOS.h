#pragma once

#include <Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h>
#include <Middlewares/Third_Party/FreeRTOS/Source/include/semphr.h>
#include <Middlewares/Third_Party/FreeRTOS/Source/include/task.h>
#include <Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h>
#include <Core/Inc/FreeRTOSConfig.h>

class RT_Task {
public:
	bool create(TaskFunction_t pvTaskCode, const char *const pcName, uint32_t usStackDepth, void *pvParameters, UBaseType_t uxPriority)
	{

		BaseType_t retval = xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, (osPriority_t)uxPriority, &handle);

		if (retval == pdPASS) {
			return true;
		}

		handle = nullptr; // just to be sure
		return false;
	}

	bool createPinnedToCore(TaskFunction_t pvTaskCode, const char *const pcName, uint32_t usStackDepth,
		void *pvParameters, UBaseType_t uxPriority, BaseType_t xCoreID)
	{

		//  BaseType_t retval = xTaskCreatePinnedToCore(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, &handle, xCoreID);
		//
		//    if (retval == pdPASS) {
		//      return true;
		//    }

		handle = nullptr; // just to be sure
		return false;
	}

	void del()
	{
		if (handle) {
			vTaskDelete(handle);
		}
	}

	RT_Task()
	{
		handle = nullptr;
	}

	~RT_Task()
	{
		this->del();
	}

	void suspend()
	{
		if (handle) {
			vTaskSuspend(handle);
		}
	}

	void resume()
	{
		if (handle) {
			vTaskResume(handle);
		}
	}

	static uint32_t thisTaskGetStackHighWaterMark()
	{
		return uxTaskGetStackHighWaterMark(NULL);
	}

	uint32_t getStackHighWaterMark()
	{
		// TaskHandle_t xHandle;
		return uxTaskGetStackHighWaterMark(handle);
#if 0

    TaskStatus_t xTaskDetails;
    vTaskGetInfo( /* The handle of the task being queried. */
                  xHandle,
                  /* The TaskStatus_t structure to complete with information
                  on xTask. */
                  &xTaskDetails,
                  /* Include the stack high water mark value in the
                  TaskStatus_t structure. */
                  pdTRUE,
                  /* Include the task state in the TaskStatus_t structure. */
                  eInvalid );
    return xTaskDetails.usStackHighWaterMark;
#endif
	}

private:
	TaskHandle_t handle;
};

class RT_PeriodicTimer {
public:
	RT_PeriodicTimer(TickType_t frec)
	{
		xFrequency = frec;
		xLastWakeTime = xTaskGetTickCount();
	}

	void init()
	{
		xLastWakeTime = xTaskGetTickCount();
	}

	void wait()
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

private:
	TickType_t xLastWakeTime;
	TickType_t xFrequency;
};

class RT_Mutex {
public:
	bool create()
	{
		handle = xSemaphoreCreateMutex();
		if (handle)
			return true;

		return false; // mutex is not created
	}

	void del()
	{
		if (handle) {
			vSemaphoreDelete(handle);
		}
	}

	RT_Mutex()
	{
		handle = nullptr;
	}

	~RT_Mutex()
	{
		this->del();
	}

	bool lock()
	{
		if (handle) {
			if (xSemaphoreTake(handle, portMAX_DELAY) == pdTRUE) {
				return true;
			}
		}
		return false;
	}

	bool lock(TickType_t xTicksToWait)
	{
		if (handle) {
			if (xSemaphoreTake(handle, xTicksToWait) == pdTRUE) {
				return true;
			}
		}
		return false;
	}

	bool unlock()
	{
		if (handle) {
			if (xSemaphoreGive(handle) == pdTRUE) {
				return true;
			}
		}
		return false;
	}

private:
	SemaphoreHandle_t handle;
};

class RT_Semaphore {
public:
	bool create(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount)
	{
		handle = xSemaphoreCreateCounting(uxMaxCount, uxInitialCount);
		if (handle)
			return true;

		return false; // semaphore is not created
	}

	void del()
	{
		if (handle) {
			vSemaphoreDelete(handle);
		}
	}

	RT_Semaphore()
	{
		handle = nullptr;
	}

	~RT_Semaphore()
	{
		this->del();
	}

	bool wait()
	{
		if (handle) {
			if (xSemaphoreTake(handle, portMAX_DELAY) == pdTRUE) {
				return true;
			}
		}
		return false;
	}

	bool wait(TickType_t xTicksToWait)
	{
		if (handle) {
			if (xSemaphoreTake(handle, xTicksToWait) == pdTRUE) {
				return true;
			}
		}
		return false;
	}

	bool signal()
	{
		if (handle) {
			if (xSemaphoreGive(handle) == pdTRUE) {
				return true;
			}
		}
		return false;
	}

private:
	SemaphoreHandle_t handle;
};

template <class T>
class MutexObject {
private:
	T object;
	RT_Mutex mutex;

public:
	MutexObject()
	{
		mutex.create();
	}
	~MutexObject()
	{
		mutex.del();
	}

	T get()
	{
		T obj;
		mutex.lock();
		obj = object;
		mutex.unlock();
		return obj;
	}

	bool set(T obj)
	{
		bool ret = mutex.lock();
		if (!ret)
			return false;
		object = obj;
		return mutex.unlock();
	}

	MutexObject &operator=(const T &o)
	{
		this->Set(o);
		return *this;
	}

	operator T()
	{
		T o;
		o = this->get();
		return o;
	}
};
