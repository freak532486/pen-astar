#pragma once

#include "graph.h"
#include "timestamp_vector.h"
#include "base/id_queue.h"
#include "base/constants.h"
#include "contraction.h"
#include "performance_logger.h"
#include "boolset.h"
#include "potentials.h"
#include <unordered_set>
#include <ctype.h>
#include <thread>
#include <mutex>

class AStarService {

	private:
		const Graph& g;
		HeuristicProvider& heur;
		std::unordered_set<node_t> closed_list;
		TimestampVector<uint32_t> dist_vec;
		TimestampVector<uint32_t> prev_vec;
		MinIDQueue queue;
		uint32_t max_dist = inf_weight;


	public:
		AStarService(const Graph& _g, HeuristicProvider& heur) : g(_g), heur(heur), dist_vec(_g.size(), inf_weight), prev_vec(_g.size(), invalid_id), queue(g.size()) {
			dist_vec.step_time();
			prev_vec.step_time();
		}

		void add_source(node_t n) {
			queue.push({ n, heur(n) });
			dist_vec.set(n, 0);
			prev_vec.set(n, invalid_id);
		}

		void set_max_dist(uint32_t max_dist) {
			this->max_dist = max_dist;
		}

		uint32_t get_dist(node_t n) {
			return dist_vec.get(n);
		}

		node_t step() {
			IDKeyPair best = queue.pop();
			const std::vector<Edge>& arcs = g.get_out_arcs(best.id);
			for (int i = 0; i < arcs.size(); i++) {
				if (closed_list.count(arcs[i].target) > 0) {
					continue;
				}
				uint32_t tentative_g = dist_vec.get(best.id) + arcs[i].weight;
				if (queue.contains_id(arcs[i].target) && tentative_g >= dist_vec.get(arcs[i].target)) {
					continue;
				}
				prev_vec.set(arcs[i].target, best.id);
				dist_vec.set(arcs[i].target, tentative_g);
				uint32_t h = heur(arcs[i].target);
				uint32_t f = tentative_g + h;
				if (f > max_dist) {
					continue;
				}
				if (queue.contains_id(arcs[i].target)) {
					if (f < queue.get_key(arcs[i].target)) {
						queue.decrease_key({ arcs[i].target, f });
					}
				} else {
					queue.push({ arcs[i].target, f });
				}
			}
			return best.id;
		}

		Path get_path(node_t target) {
			std::vector<node_t> path;
			uint32_t dist = get_dist(target);
			if (dist == inf_weight) {
				return { std::vector<node_t>(), dist };
			}
			while (target != invalid_id) {
				path.push_back(target);
				target = prev_vec.get(target);
			}
			std::reverse(path.begin(), path.end());
			return { path, dist };
		}

		void run_until_target_found(node_t target) {
			if (closed_list.count(target) > 0 || queue.empty()) { return; }
			while (!queue.empty()) {
				node_t current_node = step();
				if (current_node == target) {
					break;
				}
				closed_list.insert(current_node);
			}
			global_performance_logger.log_iteration_astar_search_space(closed_list.size());
		}

		void finish() {
			closed_list.clear();
			dist_vec.step_time();
			prev_vec.step_time();
			queue.clear();
		}
};

class BidirectionalAStarService {

private:
	const Graph& g;
	// To prevent synchronization from slowing down the search too much,
	// both threads get their own heuristic vector.
	CHPotentialService pot_f1;
	CHPotentialService pot_f2;
	ReverseCHPotentialService pot_r1;
	ReverseCHPotentialService pot_r2;
	BoolSet closed_f, closed_r;
	MinIDQueue q_f, q_r;
	TimestampVector<uint32_t> dist_f, dist_r;
	TimestampVector<node_t> par_f, par_r;
	uint32_t tentative_dist = inf_weight;
	node_t best_node = invalid_id;
	node_t source = invalid_id;
	node_t target = invalid_id;
	std::vector<std::mutex> locks;
	uint32_t min_key_f = inf_weight;
	uint32_t k_f = 0; // Top key of forward queue
	uint32_t k_r = 0;

	uint32_t heur_f1(node_t n) {
		return (pot_f1(n) + pot_r1(target) - pot_r1(n)) / 2;
	}

	uint32_t heur_r1(node_t n) {
		return (pot_r1(n) + pot_f1(source) - pot_f1(n)) / 2;
	}

	uint32_t heur_f2(node_t n) {
		return (pot_f2(n) + pot_r2(target) - pot_r2(n)) / 2;
	}

	uint32_t heur_r2(node_t n) {
		return (pot_r2(n) + pot_f2(source) - pot_f2(n)) / 2;
	}


	void step_f() {
		IDKeyPair best = q_f.pop();
		closed_f.set(best.id);
		const auto& arcs = g.get_out_arcs(best.id);
		for (const Edge& arc : arcs) {
			uint32_t g = dist_f.get(best.id) + arc.weight;
			if (g + pot_f1(arc.target) >= tentative_dist) { // Pruning
				continue;
			}
			if (closed_r.has(arc.target) && g + dist_r.get(arc.target) < tentative_dist) {
				tentative_dist = g + dist_r.get(arc.target);
				best_node = arc.target;
			}
			if (g < dist_f.get(arc.target)) {
				dist_f.set(arc.target, g);
				par_f.set(arc.target, best.id);
				uint32_t k = dist_f.get(arc.target) + heur_f1(arc.target);
				if (q_f.contains_id(arc.target)) {
					q_f.decrease_key({ arc.target, k });
				} else {
					q_f.push({ arc.target, k });
				}
			}
		}
	}

	void step_r() {
		IDKeyPair best = q_r.pop();
		closed_r.set(best.id);
		const auto& arcs = g.get_rev_out_arcs(best.id);
		for (const Edge& arc : arcs) {
			uint32_t g = dist_r.get(best.id) + arc.weight;
			if (g + pot_r2(arc.target) >= tentative_dist) { // Pruning
				continue;
			}
			if (closed_f.has(arc.target) && g + dist_f.get(arc.target) < tentative_dist) {
				tentative_dist = g + dist_f.get(arc.target);
				best_node = arc.target;
			}
			if (g < dist_r.get(arc.target)) {
				dist_r.set(arc.target, g);
				par_r.set(arc.target, best.id);
				uint32_t k = dist_r.get(arc.target) + heur_r2(arc.target);
				if (q_r.contains_id(arc.target)) {
					q_r.decrease_key({ arc.target, k });
				} else {
					q_r.push({ arc.target, k });
				}
			}
		}
	}

	void thread_function_f() {
		while (k_f + k_r < tentative_dist + heur_f1(source)) {
			step_f();
			if (!q_f.empty()) {
				k_f = q_f.peek().key;
			} else {
				k_f = inf_weight;
			}
		}
	}

	void thread_function_r() {
		while (k_f + k_r < tentative_dist + heur_f2(source)) {
			step_r();
			if (!q_r.empty()) {
				k_r = q_r.peek().key;
			} else {
				k_r = inf_weight;
			}
		}
	}

	Path get_path() {
		Path ret = { std::vector<node_t>(), tentative_dist };
		node_t current = best_node;
		while (current != invalid_id) {
			ret.nodes.push_back(current);
			current = par_f.get(current);
		}
		std::reverse(ret.nodes.begin(), ret.nodes.end());
		current = best_node;
		while (current != invalid_id) {
			current = par_r.get(current);
			ret.nodes.push_back(current);
		}
		ret.nodes.pop_back();
		return ret;
	}


public:

	BidirectionalAStarService(const Graph& g, const ContractionHierarchy& ch) :
		g(g), 
		pot_f1(ch), pot_r1(ch), pot_f2(ch), pot_r2(ch),
		q_f(g.size()), q_r(g.size()),
		closed_f(g.size()), closed_r(g.size()),
		dist_f(g.size(), inf_weight), dist_r(g.size(), inf_weight),
		par_f(g.size(), invalid_id), par_r(g.size(), invalid_id),
		locks(g.size())
	{

	}

	Path run(node_t source, node_t target) {
		this->source = source;
		this->target = target;
		pot_f1.set_target(target);
		pot_f2.set_target(target);
		pot_r1.set_target(source);
		pot_r2.set_target(source);
		dist_f.set(source, 0);
		dist_r.set(target, 0);
		par_f.set(source, invalid_id);
		par_r.set(target, invalid_id);
		q_f.push({ source, heur_f1(source) });
		q_r.push({ target, heur_r1(target) });
		min_key_f = heur_f1(source);
		closed_f.set(source);
		closed_r.set(target);
		k_f = heur_f1(source);
		k_r = heur_r1(target);
		std::thread thread_f(&BidirectionalAStarService::thread_function_f, this);
		std::thread thread_r(&BidirectionalAStarService::thread_function_r, this);
		thread_f.join();
		thread_r.join();
		global_performance_logger.log_iteration_astar_search_space(closed_f.size() + closed_r.size());
		Path ret = get_path();
		// Cleanup
		dist_f.step_time();
		dist_r.step_time();
		par_f.step_time();
		par_r.step_time();
		q_f.clear();
		q_r.clear();
		closed_f.clear();
		closed_r.clear();
		tentative_dist = inf_weight;
		best_node = invalid_id;
		return ret;
	}


};
