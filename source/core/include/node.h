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
		const std::string& getLabel() const { return label; }
		const size_t getIndex() const { return index; }
	};

}; // namespace blast
