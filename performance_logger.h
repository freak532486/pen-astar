#pragma once

#include "graph.h"
#include <string>

class PerformanceLogger {

private:

	struct IterationData {
		long apply_penalties_time = 0;
		long astar_time = 0;
		long is_feasible_time = 0;
		long total_time = 0;
		uint32_t alt_path_length = 0;
		uint32_t search_space = 0;
	};

	struct TestCaseResult {
		node_t source;
		node_t target;
		uint32_t dijkstra_rank;
		uint32_t shortest_path_length;
		std::vector<PathQualityResult> alt_path_qualities;
		long first_astar_time = 0;
		long path_extraction_time = 0;
		long total_time;
		std::vector<IterationData> iterations;
	};

	std::vector<TestCaseResult> test_case_results;
	TestCaseResult* current_case = NULL;
	IterationData* current_iteration = NULL;


public:

	void begin_test_case() {
		test_case_results.push_back(TestCaseResult());
		current_case = &test_case_results.back();
	}

	void finish_test_case() {
		current_case = NULL;
		current_iteration = NULL;
	}

	void begin_iteration() {
		if (current_case != NULL) {
			IterationData d;
			current_case->iterations.push_back(d);
			current_iteration = &(current_case->iterations.back());
		}
	}

	void end_iteration() {
		current_iteration = NULL;
	}

	void set_source(node_t source) {
		if (current_case != NULL) {
			current_case->source = source;
		}
	}

	void set_target(node_t target) {
		if (current_case != NULL) {
			current_case->target = target;
		}
	}

	void set_dijkstra_rank(uint32_t rank) {
		if (current_case != NULL) {
			current_case->dijkstra_rank = rank;
		}
	}

	void log_first_astar_time(long t) {
		if (current_case != NULL) {
			current_case->first_astar_time = t;
		}
	}

	void log_path_extraction_time(long t) {
		if (current_case != NULL) {
			current_case->path_extraction_time = t;
		}
	}

	void log_iteration_apply_penalty_time(long t) {
		if (current_iteration != NULL) {
			current_iteration->apply_penalties_time = t;
		}
	}

	void log_iteration_astar_time(long t) {
		if (current_iteration != NULL) {
			current_iteration->astar_time = t;
		}
	}

	void log_iteration_astar_search_space(uint32_t search_space) {
		if (current_iteration != NULL) {
			current_iteration->search_space = search_space;
		}
	}

	void log_iteration_is_feasible_time(long t) {
		if (current_iteration != NULL) {
			current_iteration->is_feasible_time = t;
		}
	}

	void log_iteration_alt_path_length(uint32_t length) {
		if (current_iteration != NULL) {
			current_iteration->alt_path_length = length;
		}
	}

	void log_iteration_total_runtime(long t) {
		if (current_iteration != NULL) {
			current_iteration->total_time = t;
		}
	}

	void log_total_runtime(long t) {
		if (current_case != NULL) {
			current_case->total_time = t;
		}
	}

	void log_shortest_path_length(uint32_t l) {
		if (current_case != NULL) {
			current_case->shortest_path_length = l;
		}
	}

	void log_alt_path_quality(PathQualityResult pq) {
		if (current_case != NULL) {
			current_case->alt_path_qualities.push_back(pq);
		}
	}

	std::string results_to_json_string() {
		std::string json = "{\n";
		json += "  \"tests\": {\n";
		json += "    \"cases\": [\n";
		for (auto i = test_case_results.begin(); i < test_case_results.end(); i++) {
			json += "      {\n";
			json += "        \"source\": " + std::to_string(i->source) + ",\n";
			json += "        \"target\": " + std::to_string(i->target) + ",\n";
			json += "        \"rank\": " + std::to_string(i->dijkstra_rank) + ",\n";
			json += "        \"shortest_length\": " + std::to_string(i->shortest_path_length) + ",\n";
			json += "        \"alt_paths\": [\n";
			for (int j = 0; j < i->alt_path_qualities.size(); j++) {
				uint32_t length = i->alt_path_qualities[j].length;
				float stretch = i->alt_path_qualities[j].stretch;
				float sharing = i->alt_path_qualities[j].sharing;
				float lo = i->alt_path_qualities[j].local_optimality;
				float ubs = i->alt_path_qualities[j].uniformly_bounded_stretch;
				json += "          {\n";
				json += "            \"length\": " + std::to_string(length) + ",\n";
				json += "            \"stretch\": " + std::to_string(stretch) + ",\n";
				json += "            \"sharing\": " + std::to_string(sharing) + ",\n";
				json += "            \"local_optimality\": " + std::to_string(lo) + ",\n";
				json += "            \"uniformly_bounded_stretch\": " + std::to_string(ubs) + "\n";
				json += "          }";
				if (j < i->alt_path_qualities.size() - 1) {
					json += ",\n";
				} else {
					json += "\n";
				}
			}
			json += "        ],\n";
			json += "        \"first_astar_time\": " + std::to_string(i->first_astar_time) + ",\n";
			json += "        \"path_extraction_time\": " + std::to_string(i->path_extraction_time) + ",\n";
			json += "        \"total_time\": " + std::to_string(i->total_time) + ",\n";
			json += "        \"iterations\": [\n";
			for (int j = 0; j < i->iterations.size(); j++) {
				json += "          { ";
				json += "\"apply_penalties\": " + std::to_string(i->iterations[j].apply_penalties_time) + ", ";
				json += "\"astar_time\": " + std::to_string(i->iterations[j].astar_time) + ", ";
				json += "\"astar_search_space\": " + std::to_string(i->iterations[j].search_space) + ", ";
				json += "\"is_feasible\": " + std::to_string(i->iterations[j].is_feasible_time) + ", ";
				json += "\"alt_path_length\": " + std::to_string(i->iterations[j].alt_path_length) + ", ";
				json += "\"total\": " + std::to_string(i->iterations[j].total_time) + " }";
				if (j != i->iterations.size() - 1) {
					json += ", ";
				}
				json += "\n";
			}
			json += "        ]\n";
			json += "      }";
			if (i != test_case_results.end() - 1) {
				json += ", ";
			}
			json += "\n";
		}
		json += "    ]\n";
		json += "  }\n";
		json += "}";
		return json;
	}
};

PerformanceLogger global_performance_logger = PerformanceLogger();