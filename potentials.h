#pragma once

#include "graph.h"
#include "dijkstra.h"
#include "contraction.h"
#include "timestamp_vector.h"
#include "base/constants.h"
#include <ctype.h>
#include <mutex>

class HeuristicProvider {
public:
	virtual uint32_t operator()(node_t node) = 0;
};

class CHPotentialService : public HeuristicProvider {

	private:
		const Graph& forward_graph;
		const Graph& backward_graph;
		DijkstraService backward_search;
		TimestampVector<uint32_t> potentials;
		node_t target;


	public:
		CHPotentialService(const ContractionHierarchy& ch) : forward_graph(ch.forward_graph), backward_graph(ch.backward_graph), backward_search(ch.backward_graph), potentials(ch.backward_graph.size(), inf_weight) {
			target = invalid_id;
		}

		uint32_t get_potential(node_t node) {
			if (!potentials.has(node)) {
				potentials.set(node, backward_search.get_dist(node));
				const std::vector<Edge>& up_arcs = forward_graph.get_out_arcs(node);
				for (int i = 0; i < up_arcs.size(); i++) {
					potentials.set(node, std::min(potentials.get(node), get_potential(up_arcs[i].target) + up_arcs[i].weight));
				}
			}
			return potentials.get(node);
		}

		void set_target(node_t _target) {
			backward_search.finish();
			backward_search.set_source(_target);
			backward_search.run_until_done();
			potentials.step_time();
			target = _target;
		}

		uint32_t operator()(node_t node) {
			uint32_t ret = get_potential(node);
			return ret;
		}
};

class ReverseCHPotentialService : public HeuristicProvider {

private:
	const ContractionHierarchy& ch;
	DijkstraService forward_search;
	TimestampVector<uint32_t> potentials;
	node_t target;

public:
	ReverseCHPotentialService(const ContractionHierarchy& ch) : ch(ch), forward_search(ch.forward_graph), potentials(ch.backward_graph.size(), inf_weight) {
		target = invalid_id;
	}

	uint32_t get_potential(node_t node) {
		if (!potentials.has(node)) {
			potentials.set(node, forward_search.get_dist(node));
			const std::vector<Edge>& up_arcs = ch.backward_graph.get_out_arcs(node);
			for (int i = 0; i < up_arcs.size(); i++) {
				potentials.set(node, std::min(potentials.get(node), get_potential(up_arcs[i].target) + up_arcs[i].weight));
			}
		}
		return potentials.get(node);
	}

	void set_target(node_t _target) {
		forward_search.finish();
		forward_search.set_source(_target);
		forward_search.run_until_done();
		potentials.step_time();
		target = _target;
	}

	uint32_t operator()(node_t node) {
		uint32_t ret = get_potential(node);
		return ret;
	}
};