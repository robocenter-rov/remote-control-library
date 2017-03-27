#pragma once
#include <functional>
#include "Communicator.h"

enum DELIVERY_COMMAND_STATUS {
	DCS_NONE,
	DCS_SENDING,
	DCS_SENDED,
	DCS_RESPOND_TIME_OUT,
	DCS_RESOURCE_LOCKED,
	DCS_WORKER_IS_BUSY
};

struct DeliveryCommandState_t {
	DELIVERY_COMMAND_STATUS status;
	int send_attempt;
	int resend_attempt;

	DeliveryCommandState_t() : status(DCS_NONE), send_attempt(0), resend_attempt(0) {}
	DeliveryCommandState_t(DELIVERY_COMMAND_STATUS status, int send_attempt = 0, int resend_attempt = 0) 
		: status(status), send_attempt(send_attempt), resend_attempt(resend_attempt) {}

	bool operator!=(const DeliveryCommandState_t& b) const;
};

class Task_t {
protected:
	TASK_ID _task_id;
	int _worker_id;
	unsigned int _task_tag;
	TASK_STATUS _task_status;

	std::chrono::time_point<std::chrono::system_clock> _last_state_update_time;
	std::chrono::time_point<std::chrono::system_clock> _last_request_send_time;

	virtual void _UpdateState(AdditionalData_t* additional_data) {};
public:
	virtual ~Task_t() = default;
	Task_t(TASK_ID task_id);
	class Promise_t {
	protected:
		typedef std::function<void(DeliveryCommandState_t)> on_dcs_callback_t;
		typedef std::function<void()> empty_callback_t;

		DeliveryCommandState_t _delivery_command_state;
		bool _cancelled;

		on_dcs_callback_t _on_dsc;
		empty_callback_t _on_cancel;

		void SetDeliveryCommandState(DeliveryCommandState_t delivery_command_state);
		void SetCancel();
		DeliveryCommandState_t GetDeliveryCommandState() const;

		friend Task_t;
	public:
		Promise_t();
		Promise_t* OnDeliveryStatusChanged(on_dcs_callback_t on_dsc);
		Promise_t* OnCancel(empty_callback_t on_cancel);
	};
	typedef std::shared_ptr<Promise_t> PromisePtr_t;
	virtual void SendCommand(Communicator_t* communicator, int worker_id, unsigned int tag) = 0;
	void SendStateRequest(Communicator_t* communicator);
	void UpdateDeliveryState(DeliveryCommandState_t) const;
	DeliveryCommandState_t GetDeliveryState() const;
	TASK_STATUS GetStatus() const;
	void UpdateState(TASK_STATUS task_status, AdditionalData_t* additional_data);
	void SetWorkerId(int worker_id);
	int GetWorkerid() const;
	void SetTag(unsigned int tag);
	unsigned int GetTag() const;
	PromisePtr_t GetPromise();
	std::chrono::time_point<std::chrono::system_clock> GetLastStateUpdateTime() const;
	std::chrono::time_point<std::chrono::system_clock> GetLastRequestSendTime() const;
protected:
	PromisePtr_t _promise;
};