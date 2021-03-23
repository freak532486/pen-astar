#define NOMINMAX

#include <cxxopts.hpp>
#include <aixlog.hpp>
#include "graph.h"
#include "loader.h"
#include "contraction.h"
#include "new_potentials.h"
#include "penalty.h"
#include "performance_logger.h"
#include "visualisation.h"
#include "xbdv.h"
#include <iostream>
#include <fstream>
#include <optional>
#include <random>

PathQualityResult get_path_quality(const Graph& g, const ContractionHierarchy& ch, const Path& path, float alpha) {
	PathQualityResult ret;
	ret.length = path.length;
	DijkstraService dijkstra(g);
	// Sharing
	dijkstra.set_source(path.nodes.front());
	dijkstra.run_until_target_found(path.nodes.back());
	Path optimal_path = dijkstra.get_path(path.nodes.back());
	dijkstra.finish();
	uint32_t shared_dist = 0;
	for (uint32_t i = 1; i < path.nodes.size(); i++) {
		if (std::find(optimal_path.nodes.begin(), optimal_path.nodes.end(), path.nodes[i]) != optimal_path.nodes.end()) {
			shared_dist += g.get_edge_weight(path.nodes[i - 1], path.nodes[i]);
		}
	}
	ret.sharing = (float)shared_dist / optimal_path.length;
	ret.stretch = (float)path.length / optimal_path.length;
	// Local optimality and uniformly bounded stretch (could be calculated by hand)
	CHPotentialService potentials(ch);
	float worst_ubs = 1;
	uint32_t min_dist_without_local_optimality = path.length;
	for (int i = path.nodes.size() - 1; i >= 1; i--) {
		uint32_t path_dist = 0;
		potentials.set_target(path.nodes[i]);
		for (int j = i - 1; j >= 0; j--) {
			path_dist += g.get_edge_weight(path.nodes[j], path.nodes[j + 1]);
			uint32_t optimal_dist = potentials(path.nodes[j]);
			if (path_dist != optimal_dist && path_dist < min_dist_without_local_optimality) {
				min_dist_without_local_optimality = path_dist;
			}
			float stretch = (float)path_dist / optimal_dist;
			if (stretch > worst_ubs) {
				worst_ubs = stretch;
			}
		}
	}
	ret.uniformly_bounded_stretch = worst_ubs;
	ret.local_optimality = (float)min_dist_without_local_optimality / path.length;
	return ret;
}

class ApplicationService {

private:
	const Graph& g;
	std::queue<std::tuple<node_t, node_t, uint32_t>> work_queue;
	PenaltyService penalty_service;
	const Graph* alternative_graph = NULL;
	std::optional<std::vector<float>> latitude_vec;
	std::optional<std::vector<float>> longitude_vec;
	node_t current_source = invalid_id;
	node_t current_target = invalid_id;

public:

	ApplicationService(const Graph& g, const ContractionHierarchy& ch) : g(g), penalty_service(g, ch) 
	{}

	void set_params(float alpha, float eps, float pen) {
		penalty_service.set_alpha(alpha);
		penalty_service.set_eps(eps);
		penalty_service.set_penalty_factor(pen);
	}

	void add_source_target_pair(node_t source, node_t target, uint32_t dijkstra_rank = 0) {
		work_queue.push(std::make_tuple(source, target, dijkstra_rank));
	}

	void supply_coordinate_vectors(const std::vector<float>& latitude_vec, const std::vector<float> longitude_vec) {
		this->latitude_vec.emplace(latitude_vec);
		this->longitude_vec.emplace(longitude_vec);
	}

	void run_iteration() {
		Timer timer;
		global_performance_logger.begin_test_case();
		auto current_tuple = work_queue.front();
		work_queue.pop();
		node_t source = std::get<0>(current_tuple);
		node_t target = std::get<1>(current_tuple);
		current_source = source;
		current_target = target;
		uint32_t rank = std::get<2>(current_tuple);
		LOG(INFO) << "Running Iteration: source = " << source << ", target = " << target << ", rank = " << rank << "\n";
		global_performance_logger.set_source(source);
		global_performance_logger.set_target(target);
		global_performance_logger.set_dijkstra_rank(rank);
		penalty_service.set_source(source);
		penalty_service.set_target(target);
		timer.lap();
		penalty_service.run();
		alternative_graph = &(penalty_service.get_alt_graph());
		global_performance_logger.log_total_runtime(timer.get());
	}

	std::vector<Path> extract_paths() {
		if (alternative_graph == NULL) {
			return std::vector<Path>();
		}
		XBDVService xbdv_service(*alternative_graph);
		return xbdv_service.run_bdv(current_source, current_target, false);
	}

	void save_visualisation(const std::string& path, uint32_t resolution_height = 1024) {
		if (!latitude_vec.has_value() || !longitude_vec.has_value()) {
			LOG(ERROR) << "Error: Can't visualize without latitude or longitude vector!\n";
		}
		VisualisationService vis_service(g, latitude_vec.value(), longitude_vec.value(), resolution_height);
		vis_service.clear({ 0, 0, 0 });
		vis_service.draw_graph({ 128, 128, 128 });
		vis_service.draw_subgraph(*alternative_graph, { 255, 0, 0 });
		vis_service.save(path);
	}

	void finish_iteration() {
		penalty_service.reset();
		current_source = invalid_id;
		current_target = invalid_id;
		global_performance_logger.finish_test_case();
	}

	bool is_done() {
		return work_queue.empty();
	}

	node_t get_current_source() { return current_source; }
	node_t get_current_target() { return current_target; }
};

std::pair<std::vector<uint32_t>, std::vector<uint32_t>> get_random_st_vectors(uint32_t n, uint32_t graph_size) {
	std::default_random_engine generator;
	generator.seed(time(NULL));
	std::uniform_int_distribution<uint32_t> distribution(0, graph_size);
	std::vector<uint32_t> source;
	std::vector<uint32_t> target;
	for (int i = 0; i < n; i++) {
		source.push_back(distribution(generator));
		target.push_back(distribution(generator));
	}
	return std::make_pair(source, target);
}

int run_penalty_mode(int argn, char** argv) {
	cxxopts::Options options("CH-Potentials-Penalty", "Calculates alternative routes on road network graphs using penalty method and contraction hierarchy potentials.");
	options.add_options()
		("i,input", "Path to input folder", cxxopts::value<std::string>())
		("o,output", "Path to output folder. Default: Working directory", cxxopts::value<std::string>())
		("s,source", "Source Node ID", cxxopts::value<node_t>())
		("t,target", "Target Node ID", cxxopts::value<node_t>())
		("q, quality", "Logs path quality values like uniformly bounded stretch. Takes a LOT of time.")
		("source-vector", "Path to a source vector, overrides source option", cxxopts::value<std::string>())
		("target-vector", "Path to a target vector, overrides target option", cxxopts::value<std::string>())
		("rank-vector", "Path to optional rank vector", cxxopts::value<std::string>())
		("source-limit", "Limits amount of nodes to process from source vector", cxxopts::value<uint32_t>())
		("draw-images", "Draws an image of the graph and the found alternative route graph to output folder; Requires coordinate vectors in input folder")
		("min-dijkstra-rank", "Sets minimum dijkstra rank to run and log", cxxopts::value<uint32_t>())
		("alpha", "Sets factor for rejoin penalty (default: 0.5)", cxxopts::value<float>())
		("eps", "Sets stretch value in penalty method (default: 0.1)", cxxopts::value<float>())
		("pen", "Sets penalty factor (default 0.04)", cxxopts::value<float>())
		("logname", "Sets name of log file (to prevent overwriting)", cxxopts::value<std::string>());
	;
	auto parse_result = options.parse(argn, argv);
	// Load penalty settings
	float alpha = (parse_result.count("alpha") != 0) ? parse_result["alpha"].as<float>() : 0.5;
	float eps = (parse_result.count("eps") != 0) ? parse_result["eps"].as<float>() : 0.1;
	float pen = (parse_result.count("pen") != 0) ? parse_result["pen"].as<float>() : 0.04;
	std::string logname = (parse_result.count("logname") != 0) ? parse_result["logname"].as<std::string>() : "log";
	// Load graph and ch
	std::string input_path = parse_result["input"].as<std::string>();
	if (input_path.back() != '/') {
		input_path.push_back('/');
	}
	Graph g = read_graph(input_path);
	ContractionHierarchy ch = read_ch(input_path + "ch/");
	ApplicationService executor(g, ch);
	executor.set_params(alpha, eps, pen);
	bool draw_images = false;
	bool log_quality = (parse_result.count("q") != 0);
	if (parse_result.count("draw-images") != 0) {
		draw_images = true;
		std::vector<float> latitude_vector = load_vector<float>(input_path + "latitude");
		std::vector<float> longitude_vector = load_vector<float>(input_path + "longitude");
		executor.supply_coordinate_vectors(latitude_vector, longitude_vector);
	}
	// Get output path
	std::string output_path = "./";
	if (parse_result.count("output") != 0) {
		output_path = parse_result["output"].as<std::string>();
		if (output_path.back() != '/') {
			output_path.push_back('/');
		}
	}
	// Supply sources and targets
	if (parse_result.count("source") != 0 && parse_result.count("target") != 0) {
		executor.add_source_target_pair(parse_result["source"].as<node_t>(), parse_result["target"].as<node_t>());
	} else if (parse_result.count("source-vector") != 0 && parse_result.count("target-vector") != 0) {
		std::vector<node_t> sources = load_vector<node_t>(parse_result["source-vector"].as<std::string>());
		std::vector<node_t> targets = load_vector<node_t>(parse_result["target-vector"].as<std::string>());
		std::vector<uint32_t> rank(sources.size(), 0);
		if (parse_result.count("rank-vector") != 0) {
			rank = load_vector<uint32_t>(parse_result["rank-vector"].as<std::string>());
		}
		if (sources.size() != targets.size()) {
			LOG(ERROR) << "Source and target vector don't have same size!\n";
			return 1;
		}
		if (sources.size() != rank.size()) {
			LOG(ERROR) << "Source and rank vector don't have same size!\n";
			return 1;
		}
		for (int i = 0; i < sources.size(); i++) {
			executor.add_source_target_pair(sources[i], targets[i], rank[i]);
		}
	}
	// Run
	Timer timer;
	while (!executor.is_done()) {
		executor.run_iteration();
		timer.lap();
		std::vector<Path> paths = executor.extract_paths();
		global_performance_logger.log_path_extraction_time(timer.get());
		for (const Path& path : paths) {
			if (log_quality) {
				PathQualityResult pq = get_path_quality(g, ch, path, DEFAULT_ALPHA);
				global_performance_logger.log_alt_path_quality(pq);
			} else {
				global_performance_logger.log_alt_path_quality({
					path.length, // length
					0.0,         // stretch
					0.0,         // local optimality
					0.0,         // uniformly bounded stretch
					0.0          // sharing
				});
			}
		}
		if (draw_images) {
			executor.save_visualisation(output_path + std::to_string(executor.get_current_source()) + "." + std::to_string(executor.get_current_target()) + ".ppm");
		}
		executor.finish_iteration();
	}
	// Save log file
	write_file(output_path + logname + ".json", global_performance_logger.results_to_json_string());
	return 0;
}

int generate_vectors(int argn, char** argv) {
	cxxopts::Options options("CH-Potentials-Penalty", "Calculates alternative routes on road network graphs using penalty method and contraction hierarchy potentials.");
	options.add_options()
		("i,input", "Path to input folder", cxxopts::value<std::string>())
		("o,output", "Path to output folder. Default: Working directory", cxxopts::value<std::string>())
		("s,source", "Source Node ID", cxxopts::value<node_t>())
		("source-vector", "Path to a source vector, overrides source option", cxxopts::value<std::string>())
		("limit", "Limits amount of source node", cxxopts::value<uint32_t>())
		("min-rank", "Sets minimum dijkstra rank to run and log", cxxopts::value<uint32_t>())
	;
	auto parse_result = options.parse(argn, argv);
	// Load graph
	Graph g = read_graph(parse_result["input"].as<std::string>());
	std::string output_path = parse_result["output"].as<std::string>();
	if (output_path.back() != '/') {
		output_path.push_back('/');
	}
	std::string mode(argv[2]);
	// Two modes: random or dijkstra-rank
	if (mode == "random") {
		if (parse_result.count("limit") == 0) {
			LOG(ERROR) << "Need to specify limit for random mode.\n";
			return 1;
		}
		uint32_t limit = parse_result["limit"].as<uint32_t>();
		auto st_vectors = get_random_st_vectors(limit, g.size());
		save_vector<node_t>(output_path + "source", st_vectors.first);
		save_vector<node_t>(output_path + "target", st_vectors.second);
	} else if (mode == "rank") {
		std::vector<node_t> sources;
		uint32_t min_rank = 0;
		if (parse_result.count("min-rank")) {
			min_rank = parse_result["min-rank"].as<uint32_t>();
		}
		if (parse_result.count("source") != 0) {
			sources.push_back(parse_result["source"].as<node_t>());
		} else if (parse_result.count("source-vector") != 0) {
			std::vector<node_t> input_vector = load_vector<node_t>(parse_result["source-vector"].as<std::string>());
			uint32_t limit = input_vector.size();
			if (parse_result.count("limit") != 0) {
				limit = parse_result["limit"].as<uint32_t>();
			}
			for (uint32_t i = 0; i < limit; i++) {
				sources.push_back(input_vector[i]);
			}
		} else {
			LOG(ERROR) << "You need to specify at least one source through --source or --source-vector\n";
			return 1;
		}
		std::vector<node_t> s, t; // source, target vector to save
		std::vector<uint32_t> r; // rank vector to save
		for (uint32_t i = 0; i < sources.size(); i++) {
			LOG(INFO) << "Calculating dijkstra rank nodes for source node " << i << "/" << sources.size() << "...\n";
			std::vector<node_t> rank_vector = get_dijkstra_rank_nodes(g, sources[i]);
			for (uint32_t j = min_rank; j < rank_vector.size(); j++) {
				s.push_back(sources[i]);
				t.push_back(rank_vector[j]);
				r.push_back(j);
			}
		}
		save_vector<node_t>(output_path + "source", s);
		save_vector<node_t>(output_path + "target", t);
		save_vector<uint32_t>(output_path + "rank", r);
	} else {
		LOG(ERROR) << "Unknown mode: " << mode << "\n";
		return 1;
	}
	return 0;
}

int main(int argn, char** argv) {
	AixLog::Log::init<AixLog::SinkCout>(AixLog::Severity::trace);
	if (argn < 2) {
		LOG(INFO) << "Usage: penalty [MODE] [OPTIONS]\n";
		LOG(INFO) << "See README.md for more details\n";
		return 1;
	}
	int return_code = 0;
	std::string mode(argv[1]);
	if (mode == "run") {
		return_code = run_penalty_mode(argn, argv);
	} else if (mode == "generate") {
		return_code = generate_vectors(argn, argv);
	} else {
		LOG(ERROR) << "Unknown mode: " << mode << "\n";
		return 1;
	}
	
	return 0;
}
