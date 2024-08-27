#pragma once

#include "segmenter_utility.hpp"

// Disjoint set data structure to find cycles in graphs
class DisjointSet {
public:
    DisjointSet(size_t size) : parent_(size), size_(size) {
        for (size_t idx = 0; idx < size; idx++) {
            parent_[idx] = idx;
            size_[idx] = 0;
        }
    }

    // find representative element for given x
    // using path compression
    size_t Find(size_t x);

    // combine two sets using size of sets
    void Union(size_t x, size_t y);

private:
    std::vector<size_t> parent_;
    std::vector<size_t> size_;
};
