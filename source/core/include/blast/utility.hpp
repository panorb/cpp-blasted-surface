#pragma once

namespace utility {
	template <typename T>
	struct hash_eigen {
	    std::size_t operator()(T const& matrix) const {
	        size_t hash_seed = 0;
	        for (int i = 0; i < (int)matrix.size(); i++) {
	            auto elem = *(matrix.data() + i);
	            hash_seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 +
	                         (hash_seed << 6) + (hash_seed >> 2);
	        }
	        return hash_seed;
	    }
	};
}