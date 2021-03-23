#pragma once

#include "graph.h"
#include "timestamp_vector.h"
#include "base/id_queue.h"
#include "base/constants.h"
#include <queue>
#include <utility>
#include <ctype.h>

class DijkstraService {

	private:
		const Graph& g;
		TimestampVector<node_t> dist_vec;
		TimestampVector<node_t> parent_vec;
		MinIDQueue queue;
		node_t blacklisted = invalid_id;
		uint32_t max_dist = inf_weight;
		std::vector<node_t> search_space;

	public:
		DijkstraService(const Graph& _g) : g(_g), dist_vec(_g.size(), inf_weight), parent_vec(_g.size(), invalid_id), queue(_g.size()) {}

		void set_source(node_t source) {
			dist_vec.set(source, 0);
			parent_vec.set(source, invalid_id);
			queue.push({ source, 0 });
		}

		void set_blacklisted(node_t n) {
			blacklisted = n;
		}

		void set_max_dist(uint32_t dist) {
			max_dist = dist;
		}

		node_t step() {
			node_t best = queue.pop().id;
			search_space.push_back(best);
			const std::vector<Edge>& neighbours = g.get_out_arcs(best);
			for (auto i = neighbours.begin(); i < neighbours.end(); i++) {
				if (i->target == blacklisted) {
					continue;
				}
				if (dist_vec.get(best) + i->weight < dist_vec.get(i->target)) {
					dist_vec.set(i->target, dist_vec.get(best) + i->weight);
					parent_vec.set(i->target, best);
					if (!queue.contains_id(i->target)) {
						queue.push({ i->target, dist_vec.get(i->target) });
					} else {
						queue.decrease_key({ i->target, dist_vec.get(i->target) });
					}
				}
			}
			return best;
		}

		bool is_settled(node_t n) {
			return dist_vec.has(n) && !queue.contains_id(n);
		}

		void run_until_target_found(node_t target) {
			if (is_settled(target) || queue.empty()) { return; }
			node_t cur;
			while ((cur = step()) != target) {
				if (queue.empty() || dist_vec.get(cur) >= max_dist) {
					return;
				}
			}
		}

		void run_until_done() {
			while (!queue.empty()) {
				step();
			}
		}

		uint32_t get_dist(node_t n) {
			return dist_vec.get(n);
		}

		Path get_path(node_t target) {
			std::vector<node_t> path;
			uint32_t dist = get_dist(target);
			if (dist == inf_weight) {
				return { std::vector<node_t>(), dist };
			}
			while (target != invalid_id) {
				path.push_back(target);
				target = parent_vec.get(target);
			}
			std::reverse(path.begin(), path.end());
			return { path, dist };
		}

		const std::vector<node_t>& get_search_space() {
			return search_space;
		}

		void finish() {
			dist_vec.step_time();
			queue.clear();
			search_space.clear();
			blacklisted = invalid_id;
			max_dist = inf_weight;
		}


};