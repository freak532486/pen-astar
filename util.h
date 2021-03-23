#pragma once

#include "graph.h"
#include "dijkstra.h"
#include <vector>
#include <string>
#include <fstream>

template <class T>
std::string vector_to_string(const std::vector<T>& vec) {
	if (vec.size() == 0) { return "[]"; }
	std::string ret = "[";
	for (int i = 0; i < vec.size() - 1; i++) {
		ret += std::to_string(vec[i]) + ", ";
	}
	ret += std::to_string(vec.back()) + "]";
	return ret;
}

std::vector<node_t> get_dijkstra_rank_nodes(const Graph& g, node_t source) {
	DijkstraService dij(g);
	std::vector<std::pair<uint32_t, node_t>> dist;
	std::vector<node_t> ret;
	dij.set_source(source);
	dij.run_until_done();
	for (node_t i = 0; i < g.size(); i += 1) {
		dist.push_back(std::make_pair(dij.get_dist(i), i));
	}
	std::sort(dist.begin(), dist.end());
	for (int i = 1; i < dist.size(); i *= 2) {
		ret.push_back(dist[i].second);
	}
	dij.finish();
	return ret;
}

Graph join_graphs(const Graph& a, const Graph& b) {
	if (a.size() != b.size()) { std::cerr << "Graphs not of equal size!\n"; }
	Graph ret(a.size());
	for (node_t n = 0; n < a.size(); n++) {
		const std::vector<Edge>& a_arcs = a.get_out_arcs(n);
		const std::vector<Edge>& b_arcs = b.get_out_arcs(n);
		for (int i = 0; i < a_arcs.size(); i++) {
			ret.add_edge(n, a_arcs[i]);
		}
		for (int i = 0; i < b_arcs.size(); i++) {
			ret.add_edge(b_arcs[i].target, { n, b_arcs[i].weight });
		}
	}
	return ret;
}

std::pair<Graph, Graph> split_graph(const Graph& ch, std::vector<uint32_t> ranking) {
	Graph forward_graph(ch.size());
	Graph backward_graph(ch.size());
	for (node_t n = 0; n < ch.size(); n++) {
		const std::vector<Edge>& arcs = ch.get_out_arcs(n);
		for (int j = 0; j < arcs.size(); j++) {
			if (ranking[n] < ranking[arcs[j].target]) {
				forward_graph.add_edge(n, arcs[j]);
			}
			else {
				backward_graph.add_edge(arcs[j].target, { n, arcs[j].weight });
			}
		}
	}
	return std::make_pair(forward_graph, backward_graph);
}

std::vector<uint32_t> order_to_ranking(const std::vector<node_t>& order) {
	std::vector<uint32_t> ret = std::vector<uint32_t>(order.size());
	for (uint32_t i = 0; i < order.size(); i++) {
		ret[order[i]] = i;
	}
	return ret;
}

void write_file(const std::string& path, const std::string& value) {
	std::ofstream file;
	file.open(path);
	file << value;
	file.close();
}