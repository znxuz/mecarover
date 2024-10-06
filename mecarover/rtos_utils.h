#pragma once

#include <Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h>
#include <Middlewares/Third_Party/FreeRTOS/Source/include/semphr.h>

class RtosTask {
public:
	RtosTask(TaskFunction_t pvTaskCode, const char* const pcName,
			 uint32_t usStackDepth, void* pvParameters, UBaseType_t uxPriority)
	{
		xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority,
					&this->handle);
	}

	~RtosTask() { vTaskDelete(this->handle); }

	void suspend() { vTaskSuspend(this->handle); }

	void resume() { vTaskResume(this->handle); }

	uint32_t getStackHighWaterMark()
	{
		return uxTaskGetStackHighWaterMark(this->handle);
	}

private:
	TaskHandle_t handle = nullptr;
};

class RtosPeriodicTimer {
public:
	RtosPeriodicTimer(TickType_t ticks)
		: xLastWakeTime{xTaskGetTickCount()}
		, xFrequency{ticks}
	{ }

	void wait() { vTaskDelayUntil(&xLastWakeTime, xFrequency); }

private:
	TickType_t xLastWakeTime;
	TickType_t xFrequency;
};

class RtosSemaphore {
public:
	RtosSemaphore(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount)
	{
		this->handle = xSemaphoreCreateCounting(uxMaxCount, uxInitialCount);
	}

	~RtosSemaphore() { vSemaphoreDelete(this->handle); }

	bool wait() { return xSemaphoreTake(this->handle, portMAX_DELAY); }

	bool wait(TickType_t xTicksToWait)
	{
		return xSemaphoreTake(this->handle, xTicksToWait);
	}

	bool signal() { return xSemaphoreGive(this->handle); }

private:
	SemaphoreHandle_t handle = nullptr;
};

class RtosMutex {
public:
	RtosMutex() { this->handle = xSemaphoreCreateMutex(); }

	~RtosMutex() { vSemaphoreDelete(this->handle); }

	bool lock() { return xSemaphoreTake(this->handle, portMAX_DELAY); }

	bool lock(TickType_t xTicksToWait)
	{
		return xSemaphoreTake(this->handle, xTicksToWait);
	}

	bool unlock() { return xSemaphoreGive(this->handle); }

private:
	SemaphoreHandle_t handle;
};

template<class T> class RtosMutexObject {
public:
	RtosMutexObject() = default;

	RtosMutexObject(T t) { this->set(t); }

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
	T object{};
	RtosMutex mutex{};
};
