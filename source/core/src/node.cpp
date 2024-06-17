#include "blast/node.hpp"

const std::string& blast::Node::get_label() const
{
	return label;
}

size_t blast::Node::get_index() const
{
	return index;
}
