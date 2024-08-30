#include "blast/planes/disjoint_set.hpp"


// find representative element for given x
// using path compression
size_t Disjoint_set::find(size_t x) {
    if (x != parent_[x]) {
        parent_[x] = find(parent_[x]);
    }
    return parent_[x];
}

// combine two sets using size of sets
void Disjoint_set::set_union(size_t x, size_t y) {
    x = find(x);
    y = find(y);
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
