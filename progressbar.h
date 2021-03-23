#pragma once

#include <string>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <aixlog.hpp>

constexpr long dt = 40;
constexpr int default_progress_bar_length = 30;

class ProgressBar {

private:
	uint32_t length;
	bool finished;
	bool last_print_done;
	long t;

	long time_ms() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	}

public:

	ProgressBar(int length = default_progress_bar_length) {
		this->length = length;
		this->finished = false;
		this->last_print_done = false;
		this->t = time_ms();
		update_progress(0.0);
	}

	void update_progress(double progress) {
		if (time_ms() - t < dt && !finished) {
			return;
		}
		t = time_ms();
		if (last_print_done) { return; }
		if (progress < 0) { progress = 0; }
		if (progress > 1) { progress = 1; }
		int toDraw = (int)(progress * length);
		std::setprecision(2);
		LOG(INFO) << "[" << std::string(toDraw, '#') << std::string(length - toDraw, '=') << "]  " << progress * 100 << "%" << std::string(10, ' ');
		if (finished) {
			LOG(INFO) << "\n";
			last_print_done = true;
		}
		else {
			LOG(INFO) << "\r";
		}
	}

	void finish() {
		finished = true;
		update_progress(1);
	}

	void reset() {
		finished = false;
		last_print_done = false;
	}
};