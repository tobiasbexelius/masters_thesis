#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"
#include "../../lib/include/disjoint_set.h"

namespace apm = automatic_package_measuring;

using namespace apm::internal;

TEST(DisjointSetTest, DefaultDisjointSet) {
	DisjointSet ds(10);

	std::vector<std::set<int>> sets = ds.AsSets();

	ASSERT_EQ(10, sets.size());
	for (int i = 0; i < sets.size(); ++i)
		ASSERT_EQ(1, sets[i].size());
}

TEST(DisjointSetTest, SomeOperations) {
	DisjointSet ds(10);

	ds.Union(1, 2);
	std::vector<std::set<int>> sets = ds.AsSets();
	ASSERT_EQ(9, sets.size());
	ds.Union(1, 2);
	ds.Union(2, 1);
	sets = ds.AsSets();
	ASSERT_EQ(9, sets.size());

	for (int i = 0; i < ds.Size(); ++i) {
		ds.Find(i);
	}

	sets = ds.AsSets();
	ASSERT_EQ(9, sets.size());

}

