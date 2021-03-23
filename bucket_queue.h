#pragma once

#include <unordered_map>
#include <ctype.h>
#include <climits>
#include "base/constants.h"

struct IdKeyPair {
	uint32_t id;
	int key;
};

class BucketQueue {

	private:
		std::unordered_map<int, std::vector<uint32_t>> bucket_map;
		std::vector<uint32_t> index_vec;
		std::vector<uint32_t> rank_vec;

	public:

		BucketQueue(uint32_t size) {
			index_vec = std::vector<uint32_t>(size, invalid_id);
			rank_vec = std::vector<uint32_t>(size, invalid_id);
		}

		void push(IdKeyPair p) {
			if (bucket_map.find(p.key) == bucket_map.end()) {
				bucket_map.insert(std::make_pair(p.key, std::vector<uint32_t>()));
			}
			index_vec[p.id] = bucket_map.at(p.key).size();
			rank_vec[p.id] = p.key;
			bucket_map.at(p.key).push_back(p.id);
		}

		IdKeyPair peek() {
			int smallest_key = INT_MAX;
			for (auto i = bucket_map.begin(); i != bucket_map.end(); i++) {
				if (i->first < smallest_key) {
					smallest_key = i->first;
				}
			}
			std::vector<uint32_t>& smallest_bucket = bucket_map.at(smallest_key);
			IdKeyPair ret = { smallest_bucket.back(), smallest_key };
			return ret;
		}

#pragma optimize ("", off)
		IdKeyPair pop() {
			IdKeyPair ret = peek();
			std::vector<uint32_t>& smallest_bucket = bucket_map.at(ret.key);
			smallest_bucket.pop_back();
			if (smallest_bucket.empty()) {
				bucket_map.erase(ret.key);
			}
			return ret;
		}
#pragma optimize ("", on)

		void erase_id(uint32_t id) {
			std::vector<uint32_t>& old_bucket = bucket_map.at(rank_vec[id]);
			index_vec[old_bucket.back()] = index_vec[id];
			old_bucket[index_vec[id]] = old_bucket.back();
			old_bucket.pop_back();
			if (old_bucket.empty()) {
				bucket_map.erase(rank_vec[id]);
			}
			index_vec[id] = invalid_id;
		}

		void change_key(IdKeyPair p) {
			erase_id(p.id);
			push(p);
		}

		int get_key(uint32_t id) {
			return rank_vec[id];
		}

		bool contains_id(uint32_t id) {
			if (id < 0 || id >= index_vec.size()) { return false; }
			if (index_vec[id] == invalid_id) { return false; }
			return true;
		}

		bool empty() {
			return bucket_map.empty();
		}

};
