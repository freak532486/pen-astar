#pragma once

#include <vector>
#include <utility>
#include <ctype.h>

template <class T>
class TimestampVector {

	private:
		std::vector<std::pair<T, uint32_t>> vec;
		T default_value;
		uint32_t t = 0;


	public:
		TimestampVector(uint32_t size, T default_value) : default_value(default_value) {
			vec = std::vector<std::pair<T, uint32_t>>(size);
			step_time();
		}

		void set(uint32_t index, T value) {
			vec[index].first = value;
			vec[index].second = t;
		}

		const uint32_t get(uint32_t index) {
			if (vec[index].second != t) {
				return default_value;
			} else {
				return vec[index].first;
			}
		}

		bool has(uint32_t index) {
			return vec[index].second == t;
		}

		void step_time() {
			t++;
		}


};