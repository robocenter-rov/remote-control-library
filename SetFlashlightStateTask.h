#pragma once
#include "Task.h"

class SetFlashlightStateTask_t : public Task_t {
protected:
	bool _flash_light_state;

	void _UpdateState(AdditionalData_t* additional_data) override;
public:
	SetFlashlightStateTask_t(bool state);

	class Promise_t : public Task_t::Promise_t {
	protected:
		empty_callback_t _on_ok;
		bool _ended = false;

		void SetOk();
		friend SetFlashlightStateTask_t;
	public:
		Promise_t* OnDeliveryStatusChanged(on_dcs_callback_t on_dsc) { return static_cast<Promise_t*>(Task_t::Promise_t::OnDeliveryStatusChanged(on_dsc)); }
		Promise_t* OnCancel(empty_callback_t on_cancel) { return static_cast<Promise_t*>(Task_t::Promise_t::OnCancel(on_cancel)); }
		Promise_t* OnOk(empty_callback_t on_ok);
	};
	typedef std::shared_ptr<Promise_t> PromisePtr_t;

	void SendCommand(Communicator_t* communicator, int worker_id, unsigned int tag) override;
	PromisePtr_t GetPromise();
};
