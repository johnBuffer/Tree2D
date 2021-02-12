#pragma once
#include "tree.hpp"
#include "number_generator.hpp"
#include "utils.hpp"


namespace v2
{
	struct TreeConf
	{
		float branch_width;
		float branch_width_ratio;
		float split_width_ratio;
		float branch_deviation;
		float branch_split_angle;
		float branch_split_var;
		float branch_length;
		float branch_length_ratio;
		float branch_split_proba;
		float double_split_proba;
		Vec2 attraction;
		uint32_t max_level;
	};

	struct GrowthResult
	{
		bool split;
		Node node;
		uint32_t level;
		NodeRef root;

		GrowthResult()
			: split(false)
			, node()
		{}
	};

	struct TreeBuilder
	{
		static GrowthResult grow(v2::Branch& branch, const TreeConf& conf)
		{
			GrowthResult result;
			Node& current_node = branch.nodes.back();
			const uint32_t level = branch.level;
			const uint32_t index = current_node.index;

			const float width = current_node.width;
			const float width_threshold = 0.8f;
			if (width > width_threshold) {
				// Compute new start
				const Vec2 start = current_node.getEnd();
				// Compute new length
				const float new_length = current_node.length * conf.branch_length_ratio;
				const float new_width = current_node.width * conf.branch_width_ratio;
				// Compute new direction
				const float deviation = RNGf::getRange(conf.branch_deviation);
				Vec2 direction = current_node.direction;
				direction.rotate(deviation);
				const float attraction_force = 1.0f / new_length;
				direction = (direction + conf.attraction * attraction_force).getNormalized();
				// Add new node
				branch.nodes.emplace_back(start, direction, index + 1, new_length, new_width);
				Node& new_node = branch.nodes.back();
				// Check for split
				if (index && (index % 5 == 0) && level < conf.max_level) {
					result.split = true;
					float split_angle = conf.branch_split_angle + RNGf::getRange(conf.branch_split_var);
					// Determine side
					if (RNGf::rng(0.5f)) {
						split_angle = -split_angle;
					}
					result.root.node_id = index;
					result.node.position = start;
					result.node.last_position = start;
					result.node.direction = Vec2::getRotated(direction, split_angle);
					result.node.length = new_length * conf.branch_length_ratio;
					result.node.width = new_width * conf.split_width_ratio;
					result.level = level + 1;
					result.node.index = 0;
					// Avoid single node branches
					if (result.node.width < width_threshold) {
						result.split = false;
						new_node.width = 0.0f;
					}
				}
			}

			return result;
		}

		static void grow(Tree& tree, const TreeConf& conf)
		{
			std::vector<GrowthResult> to_add;
			uint32_t i(0);
			for (Branch& b : tree.branches) {
				GrowthResult res = TreeBuilder::grow(b, conf);
				if (res.split) {
					to_add.emplace_back(res);
					to_add.back().root.branch_id = i;
				}
				++i;
			}

			for (const GrowthResult& res : to_add) {
				tree.branches[res.root.branch_id].nodes[res.root.node_id].branch_id = static_cast<uint32_t>(tree.branches.size());
				tree.branches.emplace_back(res.node, res.level);
			}
		}

		static Tree build(const Vec2& position, const TreeConf& conf)
		{
			// Create root
			float base_angle = -PI * 0.5f;
			const Node root(position, Vec2(0.0f, -1.0f), 0, conf.branch_length, conf.branch_width);
			Tree tree;
			tree.branches.emplace_back(root, 0);
			// Build the tree
			uint64_t nodes_count = 0;
			while (true) {
				grow(tree, conf);
				if (nodes_count == tree.getNodesCount()) {
					break;
				}
				nodes_count = tree.getNodesCount();
			}
			// Add physic and leaves
			tree.generateSkeleton();
			tree.addLeaves();

			return tree;
		}
	};
}
