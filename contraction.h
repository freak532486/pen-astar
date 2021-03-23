#pragma once

#include "graph.h"
#include "dijkstra.h"
#include "progressbar.h"
#include "timer.h"
#include "bucket_queue.h"
#include "boolset.h"
#include "util.h"
#include <utility>
#include <ctype.h>
#include <aixlog.hpp>

std::vector<std::pair<node_t, Edge>> shortcut_list(1000);

struct ContractionHierarchy {
	Graph forward_graph;
	Graph backward_graph;
	std::vector<node_t> ranking;
};

const std::vector<std::pair<node_t, Edge>>& get_contraction_shortcuts(const Graph& g, node_t node, DijkstraService& dijkstra_service) {
	shortcut_list.clear();
	const std::vector<Edge>& out_arcs = g.get_out_arcs(node);
	const std::vector<Edge>& rev_out_arcs = g.get_rev_out_arcs(node);
	for (int i = 0; i < rev_out_arcs.size(); i++) {
		dijkstra_service.set_blacklisted(node);
		dijkstra_service.set_source(rev_out_arcs[i].target);
		for (int j = 0; j < out_arcs.size(); j++) {
			dijkstra_service.set_max_dist(rev_out_arcs[i].weight + out_arcs[j].weight);
			dijkstra_service.run_until_target_found(out_arcs[j].target);
			if (dijkstra_service.get_dist(out_arcs[j].target) > rev_out_arcs[i].weight + out_arcs[j].weight) {
				node_t start = rev_out_arcs[i].target;
				Edge edge = { out_arcs[j].target, rev_out_arcs[i].weight + out_arcs[j].weight };
				shortcut_list.push_back(std::make_pair(start, edge));
			}
		}
		dijkstra_service.finish();
	}
	return shortcut_list;
}

BucketQueue get_initial_queue(const Graph& g) {
	int min_diff = INT_MAX;
	std::vector<uint32_t> edge_diff_vec(g.size());
	BucketQueue queue(g.size());
	ProgressBar progress_bar;
	DijkstraService dijkstra_service(g);
	for (node_t n = 0; n < g.size(); n++) {
		const auto& shortcuts = get_contraction_shortcuts(g, n, dijkstra_service);
		int edge_diff = shortcuts.size() - g.get_out_arcs(n).size() - g.get_rev_out_arcs(n).size();
		queue.push({ n, edge_diff });
		progress_bar.update_progress((double)n / g.size());
	}
	progress_bar.finish();
	return queue;
}

const std::vector<std::pair<node_t, Edge>>& contract_node(Graph& g, node_t node, DijkstraService& dijkstra_service) {
	const auto& shortcuts = get_contraction_shortcuts(g, node, dijkstra_service);
	for (int i = 0; i < shortcuts.size(); i++) {
		g.add_edge(shortcuts[i].first, shortcuts[i].second);
	}
	g.disconnect_node(node);
	return shortcuts;
}

void contract_node_and_split(Graph& g, node_t node, Graph& forward_graph, Graph& backward_graph, const std::vector<uint32_t>& ranking, DijkstraService& dijkstra_service) {
	const auto& shortcuts = get_contraction_shortcuts(g, node, dijkstra_service);
	for (int i = 0; i < shortcuts.size(); i++) {
		g.add_edge(shortcuts[i].first, shortcuts[i].second);
		if (ranking[shortcuts[i].first] < ranking[shortcuts[i].second.target]) {
			forward_graph.add_edge(shortcuts[i].first, shortcuts[i].second);
		}
		else {
			backward_graph.add_edge(shortcuts[i].second.target, { shortcuts[i].first, shortcuts[i].second.weight });
		}
	}
	g.disconnect_node(node);
}

ContractionHierarchy contract_graph(Graph& g, const std::vector<node_t>& order) {
	ProgressBar progress_bar;
	DijkstraService dijkstra_service(g);
	std::vector<uint32_t> ranking = order_to_ranking(order);
	auto g_split = split_graph(g, ranking);
	for (uint32_t i = 0; i < order.size(); i++) {
		contract_node_and_split(g, order[i], g_split.first, g_split.second, ranking, dijkstra_service);
		progress_bar.update_progress((double)i / order.size());
	}
	progress_bar.finish();
	return  { g_split.first, g_split.second, ranking };
}

ContractionHierarchy contract_by_queue(Graph& _g) {
	Graph g(_g);
	LOG(INFO) << "Calculating initial queue...\n";
	BucketQueue queue = get_initial_queue(g);
	int min_key = -(int)(queue.peek().key);
	if (min_key < 0) { min_key = 0; }
	LOG(INFO) << "Contracting Graph...\n";
	DijkstraService dijkstra_service(g);
	ProgressBar progress_bar;
	std::vector<node_t> neighbour_list(1000); // Preallocate for performance
	std::vector<uint32_t> ranking(g.size());
	int cur_rank = 0;
	while (!queue.empty()) {
		neighbour_list.clear();
		node_t best = queue.pop().id;
		ranking[best] = cur_rank;
		cur_rank++;
		const std::vector<Edge>& out_arcs = g.get_out_arcs(best);
		const std::vector<Edge>& rev_out_arcs = g.get_rev_out_arcs(best);
		for (int i = 0; i < out_arcs.size(); i++) { neighbour_list.push_back(out_arcs[i].target); }
		for (int i = 0; i < rev_out_arcs.size(); i++) { neighbour_list.push_back(rev_out_arcs[i].target); }
		const auto& shortcuts = contract_node(g, best, dijkstra_service);
		for (int i = 0; i < shortcuts.size(); i++) {
			_g.add_edge(shortcuts[i].first, shortcuts[i].second);
		}
		for (int i = 0; i < neighbour_list.size(); i++) {
			node_t neighbour = neighbour_list[i];
			if (queue.contains_id(neighbour)) {
				get_contraction_shortcuts(g, neighbour, dijkstra_service);
				int old_key = queue.get_key(neighbour);
				int new_key = shortcuts.size() - g.get_out_arcs(neighbour).size() - g.get_rev_out_arcs(neighbour).size() + 1;
				if (old_key != new_key) {
					queue.change_key({ neighbour, new_key });
				}
			}
		}
		progress_bar.update_progress((double)cur_rank / g.size());
	}
	progress_bar.finish();
	auto split = split_graph(g, ranking);
	return { split.first, split.second, ranking };
}

class CHQueryService {

private:

	const Graph& g;
	const ContractionHierarchy& ch;
	MinIDQueue forward_queue, backward_queue;
	TimestampVector<uint32_t> dist_vec_forward, dist_vec_backward;
	BoolSet forward_search_space, backward_search_space;
	uint32_t tentative_dist = inf_weight;
	node_t best_node = invalid_id;

	void step_forward() {
		node_t best = forward_queue.pop().id;
		forward_search_space.set(best);
		if (backward_search_space.has(best)) {
			if (dist_vec_forward.get(best) + dist_vec_backward.get(best) < tentative_dist) {
				tentative_dist = dist_vec_forward.get(best) + dist_vec_backward.get(best);
				best_node = best;
			}
		}
		const std::vector<Edge>& arcs = ch.forward_graph.get_out_arcs(best);
		for (const Edge& e : arcs) {
			if (dist_vec_forward.get(best) + e.weight < dist_vec_forward.get(e.target)) {
				dist_vec_forward.set(e.target, dist_vec_forward.get(best) + e.weight);
				if (!forward_queue.contains_id(e.target)) {
					forward_queue.push({ e.target, dist_vec_forward.get(e.target) });
				} else {
					forward_queue.decrease_key({ e.target, dist_vec_forward.get(e.target) });
				}
			}
		}
	}

	void step_backward() {
		node_t best = backward_queue.pop().id;
		backward_search_space.set(best);
		if (forward_search_space.has(best)) {
			if (dist_vec_forward.get(best) + dist_vec_backward.get(best) < tentative_dist) {
				tentative_dist = dist_vec_forward.get(best) + dist_vec_backward.get(best);
				best_node = best;
			}
		}
		const std::vector<Edge>& arcs = ch.backward_graph.get_out_arcs(best);
		for (const Edge& e : arcs) {
			if (dist_vec_backward.get(best) + e.weight < dist_vec_backward.get(e.target)) {
				dist_vec_backward.set(e.target, dist_vec_backward.get(best) + e.weight);
				if (!backward_queue.contains_id(e.target)) {
					backward_queue.push({ e.target, dist_vec_backward.get(e.target) });
				} else {
					backward_queue.decrease_key({ e.target, dist_vec_backward.get(e.target) });
				}
			}
		}
	}



public:

	CHQueryService(const Graph& g, const ContractionHierarchy& ch) :
		g(g),
		ch(ch),
		forward_queue(g.size()),
		backward_queue(g.size()),
		dist_vec_forward(g.size(), inf_weight), 
		dist_vec_backward(g.size(), inf_weight),
		forward_search_space(g.size()),
		backward_search_space(g.size())
	{

	}

	uint32_t query(node_t s, node_t t) {
		// Setup
		forward_queue.push({ s, 0 });
		backward_queue.push({ t, 0 });
		dist_vec_forward.set(s, 0);
		dist_vec_backward.set(t, 0);
		tentative_dist = inf_weight;
		best_node = invalid_id;
		// Query
		bool forward_done = false;
		bool backward_done = false;
		while (!forward_done || !backward_done) {
			if (!forward_done) {
				step_forward();
				if (forward_queue.empty() || forward_queue.peek().key > tentative_dist) {
					forward_done = true;
				}
			}
			if (!backward_done) {
				step_backward();
				if (backward_queue.empty() || backward_queue.peek().key > tentative_dist) {
					backward_done = true;
				}
			}
		}
		// Cleanup
		forward_queue.clear();
		backward_queue.clear();
		dist_vec_forward.step_time();
		dist_vec_backward.step_time();
		return tentative_dist;
	}

};

Path dijkstra_on_ch(node_t start, node_t end, DijkstraService& forward_service, DijkstraService& backward_service, uint32_t graph_size, bool calculate_path = true) {
	Timer timer;
	timer.lap();
	forward_service.set_source(start);
	backward_service.set_source(end);
	forward_service.run_until_done();
	backward_service.run_until_done();
	std::vector<node_t> forward_search_space = forward_service.get_search_space();
	std::vector<node_t> backward_search_space = backward_service.get_search_space();
	uint32_t best_dist = inf_weight;
	node_t best_node = invalid_id;
	timer.lap();
	for (int i = 0; i < forward_search_space.size(); i++) {
		for (int j = 0; j < backward_search_space.size(); j++) {
			if (forward_search_space[i] == backward_search_space[j]) {
				if (forward_service.get_dist(forward_search_space[i]) + backward_service.get_dist(backward_search_space[j]) < best_dist) {
					best_dist = forward_service.get_dist(forward_search_space[i]) + backward_service.get_dist(backward_search_space[j]);
					best_node = forward_search_space[i];
				}
			}
		}
	}
	Path best_path;
	best_path.length = best_dist;
	if (calculate_path) {
		best_path.nodes = forward_service.get_path(best_node).nodes;
		auto backward_path = backward_service.get_path(best_node).nodes;
		std::reverse(backward_path.begin(), backward_path.end());
		for (int i = 1; i < backward_path.size(); i++) {
			best_path.nodes.push_back(backward_path[i]);
		}
	} else {
		best_path.nodes = std::vector<node_t>();
	}
	forward_service.finish();
	backward_service.finish();
	return best_path;
}