#pragma once
#include "tree.hpp"

namespace v2
{
	namespace scaffold
	{
		struct Node
		{
			Vec2 direction;
			float length;
			uint32_t index;
			uint32_t branch_index;

			Node()
				: direction()
				, length(0.0f)
				, index(0)
				, branch_index(0)
			{}

			Node(Vec2 dir, float l, uint32_t i, uint32_t branch)
				: direction(dir)
				, length(l)
				, index(i)
				, branch_index(branch)
			{}

			Vec2 getVec() const
			{
				return direction * length;
			}
		};

		struct Branch
		{
			std::vector<Node> nodes;

			Branch(const Node& n)
				: nodes{ n }
			{}
		};

		struct Tree
		{
			std::vector<Branch> branches;
		};
	}
}

