#pragma once
#include "tree.hpp"
#include "number_generator.hpp"
#include "utils.hpp"
#include "scaffold.hpp"


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
		scaffold::Node sfd_node;
		uint32_t level;
		NodeRef root;

		GrowthResult()
			: split(false)
			, node()
			, sfd_node()
			, level(0)
			, root()
		{}
	};

	struct TreeBuilder
	{
		static GrowthResult grow(v2::scaffold::Branch& sfd_branch, v2::Branch& branch, const TreeConf& conf)
		{
			GrowthResult result;
			const scaffold::Node& sfd_node = sfd_branch.nodes.back();
			Node& current_node = branch.nodes.back();
			const uint32_t level = branch.level;
			const uint32_t index = sfd_node.index;

			const float width = current_node.width;
			const float width_threshold = 0.8f;
			if (width > width_threshold) {
				// Compute new start
				const Vec2 start = current_node.position + sfd_node.getVec();
				// Compute new length
				const float new_length = sfd_node.length * conf.branch_length_ratio;
				const float new_width = current_node.width * conf.branch_width_ratio;
				// Compute new direction
				const float deviation = RNGf::getRange(conf.branch_deviation);
				Vec2 direction = sfd_node.direction;
				direction.rotate(deviation);
				const float attraction_force = 1.0f / new_length;
				direction = (direction + conf.attraction * attraction_force).getNormalized();
				// Add new node
				branch.nodes.emplace_back(start, new_width);
				sfd_branch.nodes.emplace_back(direction, new_length, index + 1, 0);
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
					result.sfd_node.direction = Vec2::getRotated(direction, split_angle);
					result.sfd_node.length = new_length * conf.branch_length_ratio;
					result.node.width = new_width * conf.split_width_ratio;
					result.level = level + 1;
					result.sfd_node.index = 0;
					// Avoid single node branches
					if (result.node.width < width_threshold) {
						result.split = false;
						new_node.width = 0.0f;
					}
				}
			}

			return result;
		}

		static void grow(scaffold::Tree& sfd_tree, Tree& tree, const TreeConf& conf)
		{
			std::vector<GrowthResult> to_add;
			uint32_t i(0);
			for (Branch& b : tree.branches) {
				scaffold::Branch& sfd_b = sfd_tree.branches[i];
				GrowthResult res = TreeBuilder::grow(sfd_b, b, conf);
				if (res.split) {
					to_add.emplace_back(res);
					to_add.back().root.branch_id = i;
				}
				++i;
			}

			for (const GrowthResult& res : to_add) {
				//tree.branches[res.root.branch_id].nodes[res.root.node_id].branch_id = static_cast<uint32_t>(tree.branches.size());
				sfd_tree.branches.emplace_back(res.sfd_node);
				tree.branches.emplace_back(res.node, res.level, res.root.branch_id, res.root.node_id);
			}
		}

		static Tree build(Vec2 position, const TreeConf& conf)
		{
			// Create root
			const Node root(position, conf.branch_width);
			scaffold::Tree sfd_tree;
			sfd_tree.branches.emplace_back(scaffold::Node(Vec2(0.0f, -1.0f), conf.branch_length, 0, 0));
			Tree tree;
			tree.branches.emplace_back(root, 0);
			// Build the tree
			uint64_t nodes_count = 0;
			while (true) {
				grow(sfd_tree, tree, conf);
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
