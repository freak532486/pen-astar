#pragma once

#include <vector>
#include <iostream>
#include <ctype.h>
#include "base/constants.h"

typedef uint32_t node_t;

struct Edge {
	node_t target;
	uint32_t weight;
};

struct Path {
	std::vector<node_t> nodes;
	uint32_t length;

	bool operator==(const Path& b) {
		if (nodes.size() != b.nodes.size()) { return false; }
		if (length != b.length) { return false; }
		for (int i = 0; i < b.nodes.size(); i++) {
			if (nodes[i] != b.nodes[i]) {
				return false;
			}
		}
		return true;
	}
};

struct PathQualityResult {
	uint32_t length;
	float stretch;
	float local_optimality;
	float uniformly_bounded_stretch;
	float sharing;
};

class Graph {

	private:
		std::vector<std::vector<Edge>> adj_list;
		std::vector<std::vector<Edge>> rev_adj_list; // Note: Necessary for finding in-edges at contraction.

	public:

		Graph(uint32_t size) {
			adj_list = std::vector<std::vector<Edge>>(size);
			rev_adj_list = std::vector<std::vector<Edge>>(size);
		}

		const std::vector<Edge>& get_out_arcs(node_t n) const {
			return adj_list[n];
		}

		const std::vector<Edge>& get_rev_out_arcs(node_t n) const {
			return rev_adj_list[n];
		}

		void add_edge(node_t source, Edge e) {
			bool forward_found = false;
			bool backward_found = false;
			std::vector<Edge>& edges = adj_list[source];
			std::vector<Edge>& back_edges = rev_adj_list[e.target];
			for (int i = 0; i < edges.size(); i++) {
				if (edges[i].target == e.target) {
					if (edges[i].weight > e.weight) {
						edges[i].weight = e.weight;
					}
					forward_found = true;
					break;
				}
			}
			if (!forward_found) {
				adj_list[source].push_back(e);
			}
			for (int i = 0; i < back_edges.size(); i++) {
				if (back_edges[i].target == source) {
					if (back_edges[i].weight > e.weight) {
						back_edges[i].weight = e.weight;
					}
					backward_found = true;
					break;
				}
			}
			if (!backward_found) {
				rev_adj_list[e.target].push_back({ source, e.weight });
			}
		}

		bool remove_edge(node_t u, node_t v) {
			bool edge_found = false;
			for (int i = 0; i < adj_list[u].size(); i++) {
				if (adj_list[u][i].target == v) {
					Edge buffer = adj_list[u].back();
					adj_list[u][i] = buffer;
					adj_list[u].pop_back();
					edge_found = true;
					break;
				}
			}
			if (!edge_found) {
				return false;
			}
			for (int i = 0; i < rev_adj_list[v].size(); i++) {
				if (rev_adj_list[v][i].target == u) {
					Edge buffer = rev_adj_list[v].back();
					rev_adj_list[v][i] = buffer;
					rev_adj_list[v].pop_back();
					break;
				}
			}
			return true;
		}

		uint32_t get_edge_weight(node_t u, node_t v) const {
			const auto& arcs = get_out_arcs(u);
			for (auto i = arcs.begin(); i < arcs.end(); i++) {
				if (i->target == v) {
					return i->weight;
				}
			}
			return inf_weight;
		}

		void change_edge_weight(node_t u, node_t v, uint32_t new_weight) {
			auto& arcs = adj_list[u];
			for (auto i = arcs.begin(); i < arcs.end(); i++) {
				if (i->target == v) {
					i->weight = new_weight;
					break;
				}
			}
			auto& rev_arcs = rev_adj_list[v];
			for (auto i = rev_arcs.begin(); i < rev_arcs.end(); i++) {
				if (i->target == u) {
					i->weight = new_weight;
					break;
				}
			}
		}

		void disconnect_node(node_t node) {
			const auto& out_arcs = get_out_arcs(node);
			const auto& rev_out_arcs = get_rev_out_arcs(node);
			for (int i = out_arcs.size() - 1; i >= 0; i--) {
				remove_edge(node, out_arcs[i].target);
			}
			for (int i = rev_out_arcs.size() - 1; i >= 0; i--) {
				remove_edge(rev_out_arcs[i].target, node);
			}
		}

		uint32_t size() const {
			return adj_list.size();
		}

		uint32_t n_edges() const {
			uint32_t ret = 0;
			for (int i = 0; i < adj_list.size(); i++) {
				ret += adj_list[i].size();
			}
			return ret;
		}

		std::vector<std::pair<node_t, node_t>> get_edges() const {
			std::vector<std::pair<node_t, node_t>> ret;
			for (node_t i = 0; i < size(); i++) {
				const auto& out_arcs = get_out_arcs(i);
				for (const Edge& arc : out_arcs) {
					ret.push_back(std::make_pair(i, arc.target));
				}
			}
			return ret;
		}

		void clear_edges() {
			for (auto i = adj_list.begin(); i < adj_list.end(); i++) {
				if (!i->empty()) {
					i->clear();
				}
			}
			for (auto i = rev_adj_list.begin(); i < rev_adj_list.end(); i++) {
				if (!i->empty()) {
					i->clear();
				}
			}
		}

		void factor_weights(float factor) {
			for (auto& list : adj_list) {
				for (Edge& e : list) {
					e.weight = e.weight * factor;
				}
			}
			for (auto& list : rev_adj_list) {
				for (Edge& e : list) {
					e.weight = e.weight * factor;
				}
			}
		}

};