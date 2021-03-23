#pragma once

#include "potentials.h"
#include "astar.h"

class NewPotentialService : public HeuristicProvider {

private:
	CHPotentialService heur;
	TimestampVector<float> penalty;
	float factor = 1;

public:
	NewPotentialService(const ContractionHierarchy& ch) : heur(ch), penalty(ch.forward_graph.size(), 0) {}

	uint32_t operator()(node_t n) {
		return heur(n) * (factor + penalty.get(n) * (1 - factor));
	}

	void set_target(uint32_t n) {
		heur.set_target(n);
		penalty.step_time();
	}

	void penalize(node_t n) {
		if (penalty.get(n) < 1) {
			penalty.set(n, 0.1);
		}
	}

};
