#pragma once
#include "tree.hpp"

namespace v2
{
	namespace scaffold
	{
		struct Node
		{
			v2::Node node;
			Vec2 direction;
			uint32_t index;
			uint32_t branch_index;
		};
	}
}

