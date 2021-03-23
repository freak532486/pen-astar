#pragma once

#include "graph.h"
#include "base/vector_io.h"
#include "util.h"
#include "contraction.h"
#include <string>
#include <ctype.h>
#include <iostream>
#include <aixlog.hpp>

Graph read_graph(const std::string& path) {
	std::vector<uint32_t> first_out = load_vector<uint32_t>(path + "first_out");
	std::vector<uint32_t> head = load_vector<uint32_t>(path + "head");
	std::vector<uint32_t> weight = load_vector<uint32_t>(path + "weight");
	LOG(INFO) << "fo: " << first_out.size() << ", head: " << head.size() << ", w: " << weight.size() << "\n";
	Graph g(first_out.size() - 1);
	for (uint32_t i = 0; i < first_out.size() - 1; i++) {
		for (uint32_t j = first_out[i]; j < first_out[i + 1]; j++) {
			g.add_edge(i, { head[j], weight[j] });
		}
	}
	return g;
}

ContractionHierarchy read_ch(const std::string& ch_path) {
	Graph g = read_graph(ch_path);
	std::vector<uint32_t> ranking = order_to_ranking(load_vector<uint32_t>(ch_path + "order"));
	auto split = split_graph(g, ranking);
	ContractionHierarchy ret = { split.first, split.second, ranking };
	return ret;
}