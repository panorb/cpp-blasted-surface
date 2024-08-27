#include "blast/disjoint_set.hpp"


// find representative element for given x
// using path compression
size_t DisjointSet::Find(size_t x) {
    if (x != parent_[x]) {
        parent_[x] = Find(parent_[x]);
    }
    return parent_[x];
}

// combine two sets using size of sets
void DisjointSet::Union(size_t x, size_t y) {
    x = Find(x);
    y = Find(y);
    if (x != y) {
        if (size_[x] < size_[y]) {
            size_[y] += size_[x];
            parent_[x] = y;
        }
        else {
            size_[x] += size_[y];
            parent_[y] = x;
        }
    }
}
