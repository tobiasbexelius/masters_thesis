/**
 * Translated to c++ from my java code
 *
 * This class represents a disjoint-set data structure. A DisjointSet is created by passing a single
 * parameter, n, to the contstructor. Upon creation n disjoint sets are created, each containing one
 * element, which is also the representative of the set. The disjoint set can be modified with the
 * operation union(int x, int y) which unions the sets containing x and y. One can also find the
 * representative of a given set with the operation find(x).
 *
 * @author Tobias Andersson and Carl Frendin
 */
#ifndef LIB_INCLUDE_DISJOINT_SET_H_
#define LIB_INCLUDE_DISJOINT_SET_H_

#include <vector>
#include <set>

namespace automatic_package_measuring {

namespace internal {

class DisjointSet {
public:

	/**
	 * Constructs a disjoint set containing n elements. Each element is initially placed its own set
	 * of which it is the representative.
	 *
	 * @param n
	 *            the number of elements in this disjoint set
	 */
	DisjointSet(int num_elements);
	virtual ~DisjointSet();

	/**
	 * Unions the two sets which contain elements x and y. If x and y are in the same set, the same
	 * element or either does not exist, nothing happens.
	 *
	 * @param x
	 *            The first element.
	 * @param y
	 *            The second element.
	 */
	void Union(int x, int y);

	/**
	 * Finds and returns the representative of the set containing x.
	 *
	 * @param x
	 *            the element whose representative is to be searched for.
	 * @return the representative of the set containing x
	 */
	int Find(int x);

	std::vector<std::set<int>> AsSets();

	int Size();

private:
	int num_elements;
	std::vector<int> parents; // parents[i] contains the parent of element number i;
	std::vector<int> rank; // rank[i] contains the rank of element number i. This is used for balancing
	// the disjoint set.

};

} /* namespace internal */

}
/* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_DISJOINT_SET_H_ */
