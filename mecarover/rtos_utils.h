#pragma once

#include <Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h>
#include <Middlewares/Third_Party/FreeRTOS/Source/include/semphr.h>
#include <Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h>

class RT_Task {
public:
	bool create(TaskFunction_t pvTaskCode, const char *const pcName, uint32_t usStackDepth, void *pvParameters, UBaseType_t uxPriority)
	{
		auto retval = xTaskCreate(pvTaskCode, pcName, usStackDepth,
				pvParameters, (osPriority_t)uxPriority, &handle);

		if (!retval)
			handle = nullptr; // just to be sure
		return retval;
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
	}

private:
	TaskHandle_t handle;
};

class RT_PeriodicTimer {
public:
	RT_PeriodicTimer(TickType_t ticks)
		: xLastWakeTime{xTaskGetTickCount()}
		, xFrequency{ticks}
	{ }

	void wait() { vTaskDelayUntil(&xLastWakeTime, xFrequency); }

private:
	TickType_t xLastWakeTime;
	TickType_t xFrequency;
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

class RT_Mutex {
public:
	RT_Mutex() { handle = xSemaphoreCreateMutex(); }

	~RT_Mutex() { vSemaphoreDelete(handle); }

	bool lock()
	{
		return xSemaphoreTake(handle, portMAX_DELAY);
	}

	bool lock(TickType_t xTicksToWait)
	{
		return xSemaphoreTake(handle, xTicksToWait);
	}

	bool unlock()
	{
		return xSemaphoreGive(handle);
	}

private:
	SemaphoreHandle_t handle;
};

template<class T> class MutexObject {
public:
	MutexObject() = default;

	MutexObject(T t) { this->set(t); }

	T get()
	{
		T obj;
		mutex.lock();
		obj = this->object;
		mutex.unlock();
		return obj;
	}

	void set(T obj)
	{
		mutex.lock();
		this->object = obj;
		mutex.unlock();
	}

private:
	T object;
	RT_Mutex mutex;
};
