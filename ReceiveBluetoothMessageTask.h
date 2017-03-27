#pragma once
#include "Task.h"

class ReceiveBluetoothMessageTask_t : public Task_t {
protected:
	void _UpdateState(AdditionalData_t* additional_data) override;
public:
	ReceiveBluetoothMessageTask_t();

	class Promise_t : public Task_t::Promise_t {
	protected:
		typedef std::function<void(std::string)> on_receive_bluetooth_message_ok_t;

		bool _waiting = false;
		bool _ended = false;
		std::string _message;

		on_receive_bluetooth_message_ok_t _on_ok;
		empty_callback_t _on_waiting;

		void SetWaiting();
		void SetOk(std::string message);
		friend ReceiveBluetoothMessageTask_t;
	public:
		Promise_t* OnDeliveryStatusChanged(on_dcs_callback_t on_dsc) { return static_cast<Promise_t*>(Task_t::Promise_t::OnDeliveryStatusChanged(on_dsc)); }
		Promise_t* OnCancel(empty_callback_t on_cancel) { return static_cast<Promise_t*>(Task_t::Promise_t::OnCancel(on_cancel)); }
		Promise_t* OnWaiting(empty_callback_t on_waiting);
		Promise_t* OnOk(on_receive_bluetooth_message_ok_t on_ok);
	};
	typedef std::shared_ptr<Promise_t> PromisePtr_t;

	void SendCommand(Communicator_t* communicator, int worker_id, unsigned tag) override;
	PromisePtr_t GetPromise();
};
