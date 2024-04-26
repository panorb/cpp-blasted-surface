#pragma once
#include <string>

namespace blast {
	class Node {
		size_t index;
		std::string label;
		
		friend class Graph;
	public:
		Node() = default;
		Node(std::string label) : index(SIZE_MAX), label(label) {}
		const std::string& getLabel() const { return label; }
		const size_t getIndex() const { return index; }
	};

	typedef std::shared_ptr<Node> NodeP;
};
