#include "TaskManager.h"

WorkerState_t::WorkerState_t(int worker_id, WORKER_STATUS worker_status, TASK_ID task_id, unsigned int task_tag, TASK_STATUS task_status, AdditionalData_t* additional_data) 
: worker_id(worker_id), worker_status(worker_status), task_id(task_id), task_tag(task_tag), task_status(task_status), additional_data(additional_data) {}

TaskManager_t::TaskManager_t(Communicator_t* communicator) : _communicator(communicator), _command_delivery_service(communicator) {
	communicator->SetOnWorkerStateReceive([&](WorkerState_t worker_state)
	{
		_tasks_mutex.lock();
		auto task = _tasks.find(worker_state.task_tag);
		if (task != _tasks.end()) {
			task->second->UpdateState(worker_state.task_status, worker_state.additional_data);
		}
		_tasks_mutex.unlock();
	});

	communicator->SetOnLastUsedWorkerStateReceive([&](WorkerState_t worker_state) {
		_command_delivery_service.SetReceivedLastUsedWorkerState(worker_state);
	});
}

SetFlashlightStateTask_t::PromisePtr_t TaskManager_t::SetFlashlightState(bool state) {
	auto task = new SetFlashlightStateTask_t(state);
	_command_delivery_service.AddTask(task);
	return task->GetPromise();
}

ReceiveBluetoothMessageTask_t::PromisePtr_t TaskManager_t::ReadFromBluetooth() {
	auto task = new ReceiveBluetoothMessageTask_t();
	_command_delivery_service.AddTask(task);
	return task->GetPromise();
}

I2CScanTask_t::PromisePtr_t TaskManager_t::ScanI2C() {
	auto task = new I2CScanTask_t();
	_command_delivery_service.AddTask(task);
	return task->GetPromise();
}

void TaskManager_t::CommandDeliveryService_t::Update() {
	while (true) {
		_undelivered_task_queue_mutex.lock();
		_currently_delivered_task_mutex.lock();
		_currently_delivered_task = _undelivered_task_queue.front();
		_undelivered_task_queue.pop();
		_currently_delivered_task_mutex.unlock();
		_undelivered_task_queue_mutex.unlock();

		bool sended = false;
		for (int send_attempt = 0; send_attempt < 4; send_attempt++) {
			_currently_delivered_task_mutex.lock();
			_currently_delivered_task->SendCommand(_communicator, -1, get_new_task_tag());
			_currently_delivered_task->SetTag(_last_used_task_tag);
			_remote_last_used_task_tag_mutex.lock();
			_remote_last_used_task_tag = 0;
			_remote_last_used_task_tag_mutex.unlock();
			_currently_delivered_task->UpdateDeliveryState(DeliveryCommandState_t(DCS_SENDING, send_attempt, 0));
			_currently_delivered_task_mutex.unlock();

			for (int resend_attempt = 0; resend_attempt < 4; resend_attempt++) {
				_remote_last_used_task_tag_mutex.lock();
				if (_remote_last_used_task_tag > 0) {
					break;
				}
				_remote_last_used_task_tag_mutex.unlock();
				for (int i = 0; i < 100; i++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
					_currently_delivered_task_mutex.lock();
					if (sended = _currently_delivered_task->GetDeliveryState().status == DCS_SENDED) {
						break;
					}
					_currently_delivered_task_mutex.unlock();
				}
				if (sended) {
					break;
				}
				_communicator->GetLastUsedWorkerState();
				_currently_delivered_task->UpdateDeliveryState(DeliveryCommandState_t(DCS_SENDING, send_attempt, resend_attempt));
			}

			std::this_thread::yield();
		}
		_currently_delivered_task_mutex.lock();
		if (sended) {
			_delivered_task_queue_mutex.lock();
			_delivered_task_queue.push(_currently_delivered_task);
			_delivered_task_queue_mutex.unlock();
		}
		else {
			_currently_delivered_task->UpdateDeliveryState(DCS_RESPOND_TIME_OUT);
			delete _currently_delivered_task;
		}
		_currently_delivered_task = nullptr;
		_currently_delivered_task_mutex.unlock();

		std::this_thread::yield();
	}
}

TaskManager_t::CommandDeliveryService_t::CommandDeliveryService_t(Communicator_t* communicator) {
	_communicator = communicator;

	_updater = std::thread(&CommandDeliveryService_t::Update, this);
}

void TaskManager_t::CommandDeliveryService_t::AddTask(Task_t* task) {
	_undelivered_task_queue.push(task);
}

Task_t* TaskManager_t::CommandDeliveryService_t::GetDeliveredTask() {
	Task_t* res = nullptr;
	if (!_delivered_task_queue.empty()) {
		_delivered_task_queue_mutex.lock();
		res = _delivered_task_queue.front();
		_delivered_task_queue.pop();
		_delivered_task_queue_mutex.unlock();
	}
	return res;
}

void TaskManager_t::CommandDeliveryService_t::SetReceivedLastUsedWorkerState(WorkerState_t worker_state) {
	if (worker_state.task_tag == _last_used_task_tag) {
		_currently_delivered_task_mutex.lock();
		if (_currently_delivered_task == nullptr) {
			_currently_delivered_task_mutex.unlock();
			return;
		}
		_currently_delivered_task->UpdateDeliveryState(DCS_SENDED);
		_currently_delivered_task->UpdateState(worker_state.task_status, worker_state.additional_data);
		_currently_delivered_task->SetWorkerId(worker_state.worker_id);
		_delivered_task_queue_mutex.lock();
		_delivered_task_queue.push(_currently_delivered_task);
		_delivered_task_queue_mutex.unlock();
		_currently_delivered_task = nullptr;
		_currently_delivered_task_mutex.unlock();
	}
	_remote_last_used_task_tag = worker_state.task_tag;
}

void TaskManager_t::Update() {
	while (true) {
		_tasks_mutex.lock();
		while (Task_t* delivered_task = _command_delivery_service.GetDeliveredTask()) {
			_tasks[delivered_task->GetTag()] = delivered_task;
		}

		for (auto current_task = _tasks.begin(); current_task != _tasks.end(); ++current_task) {
			if (current_task->second->GetStatus() == TS_OK) {
				delete current_task->second;
				_tasks.erase(current_task++);
			}
		}

		for (auto current_task = _tasks.begin(); current_task != _tasks.end(); ++current_task) {
			if (current_task->second->GetLastStateUpdateTime() - std::chrono::system_clock::now() > std::chrono::milliseconds(5000)) {
				current_task->second->UpdateDeliveryState(DCS_RESPOND_TIME_OUT);
				delete current_task->second;
				_tasks.erase(current_task++);
			}
			if (current_task->second->GetLastStateUpdateTime() - std::chrono::system_clock::now() > std::chrono::milliseconds(100)) {
				current_task->second->SendStateRequest(_communicator);
			}
		}
		_tasks_mutex.unlock();

		std::this_thread::yield();
	}
}

unsigned TaskManager_t::CommandDeliveryService_t::get_new_task_tag() {
	return ++_last_used_task_tag;
}
