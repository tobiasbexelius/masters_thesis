#include "../include/disjoint_set.h"
#include <cassert>
#include <unordered_map>

namespace automatic_package_measuring {

namespace internal {

DisjointSet::DisjointSet(int num_elements) :
		num_elements(num_elements),
		parents(std::vector<int>(num_elements)),
		rank(std::vector<int>(num_elements)) {

	assert(num_elements > 0);

	for (int i = 0; i < num_elements; i++) {
		parents[i] = i;
	}
}

DisjointSet::~DisjointSet() {
}

void DisjointSet::Union(int x, int y) {

	assert(x < num_elements && y < num_elements && x >= 0 && y >= 0);

	int xRoot = Find(x);
	int yRoot = Find(y);
	if (xRoot == yRoot)
		return;

	// uses the rank of the representatives to determine which tree to append to which to get
	// optimal balance
	if (rank[xRoot] < rank[yRoot]) {
		parents[xRoot] = yRoot;
	} else if (rank[xRoot] > rank[yRoot]) {
		parents[yRoot] = xRoot;
	} else {
		parents[yRoot] = xRoot;
		rank[xRoot] = rank[xRoot] + 1;
	}
}

int DisjointSet::Find(int x) {

	assert(x < num_elements && x >= 0);

	if (parents[x] != x)
		parents[x] = Find(parents[x]);
	return parents[x];

}

std::vector<std::set<int> > DisjointSet::AsSets() {
	std::vector<std::set<int>> sets;
	std::unordered_map<int, int> parent_to_set;

	for (int i = 0; i < num_elements; ++i) {
		int parent = Find(i);
		if (!parent_to_set.count(parent)) {
			parent_to_set[parent] = sets.size();
			sets.push_back(std::set<int>());
		}
		sets[parent_to_set[parent]].insert(i);
	}

	return sets;
}

int DisjointSet::Size() {
	return num_elements;
}

} /* namespace internal */

} /* namespace automatic_package_measuring */
