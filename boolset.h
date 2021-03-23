#pragma once

#include "graph.h"
#include <vector>
#include <cstdint>

class BoolSet {

private:
	std::vector<bool> vec;
	std::vector<node_t> delete_list;


public:

	BoolSet(uint32_t size) {
		vec = std::vector<bool>(size, false);
	}

	void set(node_t id) {
		vec[id] = true;
		delete_list.push_back(id);
	}

	bool has(node_t id) const {
		return vec[id];
	}

	void clear() {
		while (!delete_list.empty()) {
			vec[delete_list.back()] = false;
			delete_list.pop_back();
		}
	}

	std::vector<node_t>::iterator begin() {
		return delete_list.begin();
	}

	std::vector<node_t>::iterator end() {
		return delete_list.end();
	}

	uint32_t size() const {
		return delete_list.size();
	}

};
