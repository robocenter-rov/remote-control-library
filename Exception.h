#pragma once
#include <string>

class ControllerException_t {
public:
	ControllerException_t(std::string error_message) : error_message(error_message) {}
	std::string error_message;
};
