#pragma once

#include "graph.h"
#include "dijkstra.h"
#include "base/id_queue.h"
#include <unordered_set>
#include <aixlog.hpp>

constexpr uint32_t TEST_LIMITED_SHARING = 1;
constexpr uint32_t TEST_UNIFORMLY_BOUNDED_STRETCH = 2;
constexpr uint32_t TEST_LOCAL_OPTIMALITY = 4;

constexpr float DEFAULT_ALPHA = 0.25;
constexpr float DEFAULT_GAMMA = 0.8;
constexpr float DEFAULT_EPS = 0.25;

class XBDVService {

private:
	const Graph& g;
	DijkstraService dijkstra_service;

	MinIDQueue queue_fwd;
	TimestampVector<node_t> dist_vec_fwd;
	TimestampVector<node_t> parent_vec_fwd;
	std::unordered_set<node_t> search_space_fwd;

	MinIDQueue queue_bwd;
	TimestampVector<node_t> dist_vec_bwd;
	TimestampVector<node_t> parent_vec_bwd;
	std::unordered_set<node_t> search_space_bwd;

	bool test_limited_sharing(const Path& path, const Path& optimal_path, float gamma) {
		return get_sharing(path, optimal_path) < gamma * optimal_path.length;
	}

	bool test_uniformly_bounded_stretch(const Path& path, float eps) {
		uint32_t path_dist = 0;
		node_t a, b;
		for (uint32_t i = 0; i < path.nodes.size(); i++) {
			a = path.nodes[i];
			dijkstra_service.set_source(a);
			for (uint32_t j = i + 1; j < path.nodes.size(); j++) {
				b = path.nodes[j];
				path_dist += g.get_edge_weight(path.nodes[j - 1], path.nodes[j]);
				dijkstra_service.run_until_target_found(b);
				if (dijkstra_service.get_dist(b) * (1 + eps) < path_dist) {
					return false;
				}
			}
			dijkstra_service.finish();
		}
		return true;
	}

	bool test_local_optimality(const Path& path, uint32_t max_range) {
		uint32_t path_dist = 0;
		node_t a, b;
		for (uint32_t i = 0; i < path.nodes.size(); i++) {
			a = path.nodes[i];
			dijkstra_service.set_source(a);
			for (uint32_t j = i + 1; j < path.nodes.size(); j++) {
				b = path.nodes[j];
				path_dist += g.get_edge_weight(path.nodes[j - 1], path.nodes[j]);
				if (path_dist > max_range) {
					break;
				} else {
					dijkstra_service.run_until_target_found(b);
					if (dijkstra_service.get_dist(b) < path_dist) {
						return false;
					}
				}
			}
			dijkstra_service.finish();
		}
		return true;
	}

	// Called T-Test in paper
	bool test_local_optimality_approximation(node_t via_node, uint32_t t) {
		uint32_t xy_dist = 0;
		// Find x and y nodes
		node_t x = via_node;
		uint32_t dist_to_v = 0;
		while (dist_to_v < t) {
			node_t new_x = parent_vec_fwd.get(x);
			if (new_x == invalid_id) {
				break;
			}
			dist_to_v += g.get_edge_weight(new_x, x);
			x = new_x;
		}
		xy_dist += dist_to_v;
		node_t y = via_node;
		dist_to_v = 0;
		while (dist_to_v < t) {
			node_t new_y = parent_vec_bwd.get(y);
			if (new_y == invalid_id) {
				break;
			}
			dist_to_v += g.get_edge_weight(y, new_y);
			y = new_y;
		}
		xy_dist += dist_to_v;
		// Run check for optimality
		dijkstra_service.set_source(x);
		dijkstra_service.run_until_target_found(y);
		bool ret = (dijkstra_service.get_dist(y) == xy_dist);
		dijkstra_service.finish();
		return ret;
	}

	node_t step_forward_search() {
		node_t best = queue_fwd.pop().id;
		search_space_fwd.insert(best);
		const std::vector<Edge>& arcs = g.get_out_arcs(best);
		for (const Edge& e : arcs) {
			if (dist_vec_fwd.get(e.target) > dist_vec_fwd.get(best) + e.weight) {
				dist_vec_fwd.set(e.target, dist_vec_fwd.get(best) + e.weight);
				parent_vec_fwd.set(e.target, best);
				if (!queue_fwd.contains_id(e.target)) {
					queue_fwd.push({ e.target, dist_vec_fwd.get(e.target) });
				} else {
					queue_fwd.decrease_key({ e.target, dist_vec_fwd.get(e.target) });
				}
			}
		}
		return best;
	}

	node_t step_backward_search() {
		node_t best = queue_bwd.pop().id;
		search_space_bwd.insert(best);
		const std::vector<Edge>& arcs = g.get_rev_out_arcs(best);
		for (const Edge& e : arcs) {
			if (dist_vec_bwd.get(e.target) > dist_vec_bwd.get(best) + e.weight) {
				dist_vec_bwd.set(e.target, dist_vec_bwd.get(best) + e.weight);
				parent_vec_bwd.set(e.target, best);
				if (!queue_bwd.contains_id(e.target)) {
					queue_bwd.push({ e.target, dist_vec_bwd.get(e.target) });
				} else {
					queue_bwd.decrease_key({ e.target, dist_vec_bwd.get(e.target) });
				}
			}
		}
		return best;
	}

	void run_dijkstra_bidirectional(node_t source, node_t target, uint32_t max_dist) {
		queue_fwd.push({ source, 0 });
		dist_vec_fwd.set(source, 0);
		parent_vec_fwd.set(source, invalid_id);
		queue_bwd.push({ target, 0 });
		dist_vec_bwd.set(target, 0);
		parent_vec_bwd.set(target, invalid_id);
		while (!queue_fwd.empty() || !queue_bwd.empty()) {
			if (!queue_fwd.empty()) {
				node_t forward_best = step_forward_search();
				if (dist_vec_fwd.get(forward_best) > max_dist) {
					queue_fwd.clear();
				}
			}
			if (!queue_bwd.empty()) {
				node_t backward_best = step_backward_search();
				if (dist_vec_bwd.get(backward_best) > max_dist) {
					queue_bwd.clear();
				}
			}
		}
	}

	uint32_t get_sharing(const Path& path, const Path& comp) {
		uint32_t shared_dist = 0;
		for (uint32_t i = 1; i < path.nodes.size(); i++) {
			if (std::find(comp.nodes.begin(), comp.nodes.end(), path.nodes[i]) != comp.nodes.end()) {
				shared_dist += g.get_edge_weight(path.nodes[i - 1], path.nodes[i]);
			}
		}
		return shared_dist;
	}

	uint32_t get_plateau_length(const Path& path) {
		uint32_t current_plateau_length = 0;
		uint32_t max_plateau_length = 0;
		bool in_plateau = false;
		for (uint32_t i = 0; i < path.nodes.size(); i++) {
			if (search_space_fwd.count(path.nodes[i]) > 0 && search_space_bwd.count(path.nodes[i])) {
				if (!in_plateau) {
					in_plateau = true;
				} else {
					current_plateau_length += g.get_edge_weight(path.nodes[i - 1], path.nodes[i]);
				}
			} else {
				in_plateau = false;
				if (current_plateau_length > max_plateau_length) {
					max_plateau_length = current_plateau_length;
				}
				current_plateau_length = 0;
			}
		}
		if (current_plateau_length > max_plateau_length) {
			max_plateau_length = current_plateau_length;
		}
		return max_plateau_length;
	}

	uint32_t sort_function(const Path& p, const Path& optimal_path) {
		return 2 * p.length + get_sharing(p, optimal_path) - get_plateau_length(p);
	}

	void sort_paths(std::vector<Path>& paths, const Path& optimal_path) {
		for (uint32_t i = 0; i < paths.size(); i++) {
			uint32_t min_index = 0;
			uint32_t min_key = inf_weight;
			for (uint32_t j = i; j < paths.size(); j++) {
				uint32_t key = sort_function(paths[j], optimal_path);
				if (key < min_key) {
					min_key = key;
					min_index = j;
				}
			}
			Path buffer = paths[i];
			paths[i] = paths[min_index];
			paths[min_index] = buffer;
		}
	}

	void run_forward_search(node_t source, uint32_t max_dist) {
		queue_fwd.push({ source, 0 });
		dist_vec_fwd.set(source, 0);
		parent_vec_fwd.set(source, invalid_id);
		search_space_fwd.clear();
		while (!queue_fwd.empty()) {
			node_t best = queue_fwd.pop().id;
			if (dist_vec_fwd.get(best) > max_dist) {
				break;
			} else {
				search_space_fwd.insert(best);
			}
			if (search_space_bwd.count(best) != 0) {
				continue;
			}
			const std::vector<Edge>& arcs = g.get_out_arcs(best);
			for (const Edge& e : arcs) {
				if (dist_vec_fwd.get(e.target) > dist_vec_fwd.get(best) + e.weight) {
					dist_vec_fwd.set(e.target, dist_vec_fwd.get(best) + e.weight);
					parent_vec_fwd.set(e.target, best);
					if (!queue_fwd.contains_id(e.target)) {
						queue_fwd.push({ e.target, dist_vec_fwd.get(e.target) });
					} else {
						queue_fwd.decrease_key({ e.target, dist_vec_fwd.get(e.target) });
					}
				}
			}
		}
	}

	void run_backward_search(node_t target, uint32_t max_dist) {
		queue_bwd.push({ target, 0 });
		dist_vec_bwd.set(target, 0);
		parent_vec_bwd.set(target, invalid_id);
		search_space_bwd.clear();
		while (!queue_bwd.empty()) {
			node_t best = queue_bwd.pop().id;
			if (search_space_fwd.count(best) != 0) {
				continue;
			}
			if (dist_vec_bwd.get(best) > max_dist) {
				break;
			} else {
				search_space_bwd.insert(best);
			}
			const std::vector<Edge>& arcs = g.get_rev_out_arcs(best);
			for (const Edge& e : arcs) {
				if (dist_vec_bwd.get(e.target) > dist_vec_bwd.get(best) + e.weight) {
					dist_vec_bwd.set(e.target, dist_vec_bwd.get(best) + e.weight);
					parent_vec_bwd.set(e.target, best);
					if (!queue_bwd.contains_id(e.target)) {
						queue_bwd.push({ e.target, dist_vec_bwd.get(e.target) });
					} else {
						queue_bwd.decrease_key({ e.target, dist_vec_bwd.get(e.target) });
					}
				}
			}
		}
	}

	Path get_implicit_path(node_t n) {
		uint32_t path_length = dist_vec_fwd.get(n) + dist_vec_bwd.get(n);
		std::vector<node_t> path_nodes;
		node_t current = n;
		while (current != invalid_id) {
			path_nodes.push_back(current);
			current = parent_vec_fwd.get(current);
		}
		std::reverse(path_nodes.begin(), path_nodes.end());
		current = parent_vec_bwd.get(n);
		while (current != invalid_id) {
			path_nodes.push_back(current);
			current = parent_vec_bwd.get(current);
		}
		return { path_nodes, path_length };
	}


public:

	XBDVService(const Graph& g) : 
		g(g), 
		dijkstra_service(g),
		queue_fwd(g.size()),
		dist_vec_fwd(g.size(), inf_weight),
		parent_vec_fwd(g.size(), invalid_id),
		queue_bwd(g.size()),
		dist_vec_bwd(g.size(), inf_weight),
		parent_vec_bwd(g.size(), invalid_id)
	{}

	std::vector<Path> run_bdv(node_t source, node_t target, bool run_t_test = true, float alpha = DEFAULT_ALPHA, float eps = DEFAULT_EPS, float gamma = DEFAULT_GAMMA) {
		// Get optimal path distance
		dijkstra_service.set_source(source);
		dijkstra_service.run_until_target_found(target);
		Path optimal_path = dijkstra_service.get_path(target);
		dijkstra_service.finish();
		LOG(INFO) << "Optimal path length: " << optimal_path.length << "\n";
		// Run forward and backward search
		run_dijkstra_bidirectional(source, target, optimal_path.length * (1 + eps));
		// Calculate paths for every node in the search space cut
		std::vector<node_t> search_space_cut;
		for (const node_t& n : search_space_fwd) {
			if (search_space_bwd.count(n) != 0) {
				if (dist_vec_fwd.get(n) + dist_vec_bwd.get(n) < (1 + eps) * optimal_path.length) {
					search_space_cut.push_back(n);
				}
			}
		}
		// Trim to viable paths
		std::vector<Path> alternative_paths;
		std::vector<Path> considiered_paths;
		uint32_t sharing_success = 0;
		uint32_t local_optimality_success = 0;
		for (const node_t& via_node : search_space_cut) {
			Path path = get_implicit_path(via_node);
			if (std::find(considiered_paths.begin(), considiered_paths.end(), path) == considiered_paths.end()) {
				considiered_paths.push_back(path);
			} else {
				continue;
			}
			if (!test_limited_sharing(path, optimal_path, gamma)) {
				continue;
			} else {
				sharing_success++;
			}
			if (run_t_test && !test_local_optimality_approximation(via_node, alpha * optimal_path.length)) {
				continue;
			} else {
				local_optimality_success++;
			}
			alternative_paths.push_back(path);
		}
		sort_paths(alternative_paths, optimal_path);
		LOG(INFO) << "There are " << considiered_paths.size() << " possible paths\n";
		LOG(INFO) << sharing_success << " paths passed sharing test\n";
		LOG(INFO) << local_optimality_success << " paths passed T-test\n";
		// Cleanup
		return alternative_paths;
	}

};
