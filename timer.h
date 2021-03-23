#pragma once

#include <chrono>

class Timer {

	private:
		long long t;

	public:
		void lap() {
			t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		}

		long long get() {
			return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - t;
		}
};