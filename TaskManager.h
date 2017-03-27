#pragma once
#include "SetFlashlightStateTask.h"
#include "ReceiveBluetoothMessageTask.h"
#include "I2CScanTask.h"
#include "Communicator.h"
#include <map>
#include <queue>

class command_already_sending : public std::exception {};

class TaskManager_t {
public:
	TaskManager_t(Communicator_t* _communicator);

	SetFlashlightStateTask_t::PromisePtr_t SetFlashlightState(bool state);
	ReceiveBluetoothMessageTask_t::PromisePtr_t ReadFromBluetooth();
	I2CScanTask_t::PromisePtr_t ScanI2C();

protected:
	Communicator_t* _communicator;

	std::map<unsigned int, Task_t*> _tasks;
	std::mutex _tasks_mutex;

	class CommandDeliveryService_t {
	private:
		std::queue<Task_t*> _undelivered_task_queue;
		Task_t* _currently_delivered_task;
		std::queue<Task_t*> _delivered_task_queue;

		Communicator_t* _communicator;
		unsigned int _last_used_task_tag = 0;
		unsigned int _remote_last_used_task_tag = 0;
		unsigned int get_new_task_tag();

		std::mutex _delivered_task_queue_mutex;
		std::mutex _undelivered_task_queue_mutex;
		std::mutex _currently_delivered_task_mutex;
		std::mutex _remote_last_used_task_tag_mutex;
		std::thread _updater;

		void Update();
	public:
		CommandDeliveryService_t(Communicator_t* communicator);
		void AddTask(Task_t* task);
		Task_t* GetDeliveredTask();
		void SetReceivedLastUsedWorkerState(WorkerState_t worker_state);
	} _command_delivery_service;

	std::thread _updater;

	void Update();
};
