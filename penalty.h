#pragma once

#include "graph.h"
#include "astar.h"
#include "potentials.h"
#include "new_potentials.h"
#include "timer.h"
#include "util.h"
#include "performance_logger.h"
#include "timestamp_vector.h"
#include "base/id_queue.h"
#include "base/constants.h"
#include "boolset.h"
#include <unordered_set>
#include <cmath>

class PenaltyService {

private:

	const uint32_t max_iterations = 20;

	const Graph& g;
	Graph penalized_graph;
	Graph alt_graph;
	DijkstraService alt_graph_dijkstra;
	const ContractionHierarchy& ch;
	node_t source, target;
	BidirectionalAStarService astar;
	uint32_t best_path_length;
	BoolSet node_set;

	float penalty_factor = 0.04;
	float alpha = 0.5;
	float eps = 0.1;
	float delta = 0.1;

	struct Detour {
		node_t a;
		node_t b;
		uint32_t length;
	};

	uint32_t get_real_path_length(const Path& path) {
		uint32_t ret = 0;
		for (int i = 0; i < path.nodes.size() - 1; i++) {
			uint32_t w = g.get_edge_weight(path.nodes[i], path.nodes[i + 1]);
			if (w == inf_weight) {
				return inf_weight;
			}
			ret = ret + w;
		}
		return ret;
	}

	Path get_shortest_path() {
		Path path = astar.run(source, target);
		if (path.length == inf_weight) {
			return path;
		}
		Path ret;
		ret.nodes = path.nodes;
		ret.length = path.length;
		return ret;
	}

	void add_path_to_graph(const Path& path, Graph& g) {
		for (int i = 0; i < path.nodes.size() - 1; i++) {
			node_t a = path.nodes[i];
			node_t b = path.nodes[i + 1];
			g.add_edge(a, { b, this->g.get_edge_weight(a, b) });
		}
	}

#ifdef PENALIZE_ALT_GRAPH
	void apply_penalties() {
		for (const auto& e : alt_graph.get_edges()) {
			node_t u = e.first;
			node_t v = e.second;
			penalized_graph.change_edge_weight(u, v, penalized_graph.get_edge_weight(u, v) * (1 + penalty_factor));
		}
	}
#else
	void apply_penalties(const Path& path, uint32_t optimal_path_length) {
		// Penalize path edges
		for (uint32_t i = 0; i < path.nodes.size() - 1; i++) {
			node_t u = path.nodes[i];
			node_t v = path.nodes[i + 1];
			penalized_graph.change_edge_weight(u, v, penalized_graph.get_edge_weight(u, v) * (1 + penalty_factor));
		}
		// Penalize rejoin edges (incoming)
		uint32_t rejoin_penalty = alpha * std::sqrt(optimal_path_length);
		for (uint32_t i = 0; i < path.nodes.size(); i++) {
			node_t v = path.nodes[i];
			auto& rev_out_arcs = penalized_graph.get_rev_out_arcs(v);
			for (const auto& edge : rev_out_arcs) {
				node_t u = edge.target;
				if (i == 0 || u != path.nodes[i - 1]) {
					penalized_graph.change_edge_weight(u, v, penalized_graph.get_edge_weight(u, v) + rejoin_penalty);
				}
			}
		}
	}
#endif

	std::vector<node_t> get_path_intersection(const Path& path, const Path& comp) {
		node_set.clear();
		for (auto i = comp.nodes.begin(); i < comp.nodes.end(); i++) {
			node_set.set(*i);
		}
		std::vector<node_t> ret;
		for (auto i = path.nodes.begin(); i < path.nodes.end(); i++) {
			if (node_set.has(*i)) {
				ret.push_back(*i);
			}
		}
		return ret;
	}

	std::vector<Detour> get_detours(const Path& path, const Path& comp) {
		std::vector<Detour> ret;
		std::vector<node_t> intersection = get_path_intersection(path, comp);
		node_t detour_start = invalid_id;
		node_t detour_end = invalid_id;
		uint32_t detour_dist = 0;
		bool in_detour = false;
		int intersection_index = 0;
		for (int i = 0; i < path.nodes.size(); i++) {
			if (!in_detour) {
				if (path.nodes[i] != intersection[intersection_index]) {
					in_detour = true;
					detour_start = path.nodes[i - 1];
					detour_dist = g.get_edge_weight(path.nodes[i - 1], path.nodes[i]);
				} else {
					intersection_index++;
				}
			}
			else {
				if (path.nodes[i] == intersection[intersection_index]) {
					in_detour = false;
					detour_dist += g.get_edge_weight(path.nodes[i - 1], path.nodes[i]);
					detour_end = path.nodes[i];
					ret.push_back({ detour_start, detour_end, detour_dist });
				} else {
					detour_dist += g.get_edge_weight(path.nodes[i - 1], path.nodes[i]);
				}
			}
		}
		return ret;
	}

	bool is_feasible(const Path& path, const Path& orig_path) {
		if (path.length == inf_weight) {
			return false;
		}
		std::vector<Detour> detours = get_detours(path, orig_path);
		bool found_long_enough_detour = false;
		bool found_good_enough_detour = false;
		for (auto i = detours.begin(); i < detours.end(); i++) {
			if (i->length >= delta * orig_path.length) {
				found_long_enough_detour = true;
				alt_graph_dijkstra.set_source(i->a);
				alt_graph_dijkstra.run_until_target_found(i->b);
				if (i->length <= (1 + eps) * alt_graph_dijkstra.get_dist(i->b)) {
					found_good_enough_detour = true;
				}
				alt_graph_dijkstra.finish();
			}
		}
		return found_long_enough_detour && found_good_enough_detour;
	}

public:

	PenaltyService(const Graph& g, const ContractionHierarchy& ch) : 
		g(g), 
		penalized_graph(g), 
		alt_graph(g.size()), 
		alt_graph_dijkstra(alt_graph), 
		ch(ch),
		astar(penalized_graph, ch), 
		node_set(g.size()) 
	{
		source = invalid_id;
		target = invalid_id;
	}

	void set_source(node_t n) {
		source = n;
	}

	void set_target(node_t n) {
		target = n;
	}

	void set_alpha(float alpha) {
		this->alpha = alpha;
	}
	void set_eps(float eps) {
		this->eps = eps;
	}

	void set_penalty_factor(float pen) {
		this->penalty_factor = pen;
	}


	void run() {
		Timer timer;
		Timer total_timer;
		timer.lap();
		Path original_path = get_shortest_path();
		global_performance_logger.log_first_astar_time(timer.get());
		global_performance_logger.log_shortest_path_length(original_path.length);
		add_path_to_graph(original_path, alt_graph);
		Path alt_path = original_path;
		uint32_t iterations = 0;
		while (alt_path.length <= (1 + eps) * original_path.length && iterations < max_iterations) {
			global_performance_logger.begin_iteration();
			timer.lap();
			total_timer.lap();
			#ifdef PENALIZE_ALT_GRAPH
				apply_penalties();
			#else
				apply_penalties(alt_path, original_path.length);
			#endif
			global_performance_logger.log_iteration_apply_penalty_time(timer.get());
			timer.lap();
			alt_path = get_shortest_path();
			#ifdef BREAK_ON_ORIGINAL
				alt_path.length = get_real_path_length(alt_path);
			#endif
			global_performance_logger.log_iteration_astar_time(timer.get());
			global_performance_logger.log_iteration_alt_path_length(alt_path.length);
			timer.lap();
			if (is_feasible(alt_path, original_path)) {
				add_path_to_graph(alt_path, alt_graph);
			}
			global_performance_logger.log_iteration_is_feasible_time(timer.get());
			iterations++;
			global_performance_logger.log_iteration_total_runtime(total_timer.get());
			global_performance_logger.end_iteration();
		}
	}

	const Graph& get_alt_graph() {
		return alt_graph;
	}

	void reset() {
		penalized_graph = g;
		alt_graph.clear_edges();
		source = invalid_id;
		target = invalid_id;
	}
};