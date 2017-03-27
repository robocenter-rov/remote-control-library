#pragma once

#include <algorithm>
#include <codecvt>

template<typename T>
inline T constrain(T val, T min, T max) {
	return std::max(std::min(val, max), min);
}

inline uint16_t swap_endian(const uint16_t val) {
	return val >> 8 | val << 8 & 0xFF00;
}

inline void set_le_uint16(void* buffer, const uint16_t val) {
	*reinterpret_cast<uint16_t*>(buffer) = swap_endian(val);
}

inline uint16_t get_le_uint16(const uint16_t val) {
	return swap_endian(val);
}

inline uint16_t get_be_uint16(const void* buffer) {
	const uint16_t t = *reinterpret_cast<const uint16_t*>(buffer);
	return swap_endian(t);
}

inline int16_t float_to_int(float val) {
	return constrain(val, -1.0f, 1.0f) * (INT16_MAX - 1);
}

inline float int_to_float(const int16_t val) {
	return val / INT16_MAX;
}

inline float int_to_float(const void* buffer) {
	return int_to_float(*static_cast<const int16_t*>(buffer));
}
