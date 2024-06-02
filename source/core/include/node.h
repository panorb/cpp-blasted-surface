#pragma once
#include <string>
#include <memory>

namespace blast {
	class Node {
	protected:
		size_t index;
		std::string label;
		
		friend class Graph;
	public:
		Node() = default;
		Node(std::string label) : index(SIZE_MAX), label(label) {}
		const std::string& get_label() const;
		size_t get_index() const;
	};

}; // namespace blast
