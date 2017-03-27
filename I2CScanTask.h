#pragma once
#include "Task.h"

class I2CScanTask_t : public Task_t {
public:
	I2CScanTask_t();

	class Promise_t : public Task_t::Promise_t {
	protected:
		typedef std::function<void(std::vector<char>)> on_i2c_scan_ok_callback_t;

		bool _scanning = false;
		bool _ended = false;
		std::vector<char> _adresses;

		on_i2c_scan_ok_callback_t _on_ok;
		empty_callback_t _on_scanning;

		void SetScanning();
		void SetOk(std::vector<char> adresses);
	public:
		Promise_t* OnDeliveryStatusChanged(on_dcs_callback_t on_dsc) { return static_cast<Promise_t*>(Task_t::Promise_t::OnDeliveryStatusChanged(on_dsc)); }
		Promise_t* OnCancel(empty_callback_t on_cancel) { return static_cast<Promise_t*>(Task_t::Promise_t::OnCancel(on_cancel)); }
		Promise_t* OnScanning(empty_callback_t on_scanning);
		Promise_t* OnOk(on_i2c_scan_ok_callback_t on_ok);
	};
	typedef std::shared_ptr<Promise_t> PromisePtr_t;

	void SendCommand(Communicator_t* communicator, int worker_id, unsigned tag) override;
	PromisePtr_t GetPromise();
};
