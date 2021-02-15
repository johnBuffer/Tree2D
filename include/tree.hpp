#pragma once
#include "vec2.hpp"
#include "pinned_segment.hpp"
#include "wind.hpp"
#include "utils.hpp"


namespace v2
{
	struct PhysicSegment
	{
		Vec2 attach_point;
		Vec2 direction;
		Particule moving_point;
		float length;
		float delta_angle;
		float last_angle;

		PhysicSegment()
			: length(0.0f)
			, delta_angle(0.0f)
			, last_angle(0.0f)
		{}

		PhysicSegment(Vec2 attach, Vec2 moving)
			: attach_point(attach)
			, direction((moving - attach).getNormalized())
			, moving_point(moving)
			, length((moving - attach).getLength())
			, delta_angle(0.0f)
			, last_angle(direction.getAngle())
		{
		}

		void translate(const Vec2& v)
		{
			attach_point += v;
			moving_point.position += v;
			moving_point.old_position += v;
		}

		void updateDeltaAngle()
		{
			const float new_angle = (moving_point.position - attach_point).getAngle();
			delta_angle = new_angle - last_angle;
			last_angle = new_angle;
		}

		void solveAttach()
		{
			const Vec2 delta = moving_point.position - attach_point;
			const float dist = delta.getLength();
			const float dist_delta = length - dist;
			const float inv_dist = 1.0f / dist;
			moving_point.move(Vec2(delta.x * inv_dist * dist_delta, delta.y * inv_dist * dist_delta));
		}

		void update(float dt)
		{
			solveAttach();
			moving_point.acceleration += direction;
			moving_point.update(dt);
			updateDeltaAngle();
		}
	};

	struct Node
	{
		Vec2 position;
		float width;

		Node()
			: position()
			, width(1.0f)
		{}

		Node(Vec2 pos, float w)
			: position(pos)
			, width(w)
		{}
	};

	struct NodeRef
	{
		uint32_t branch_id;
		uint32_t node_id;
		Vec2 position;

		NodeRef()
			: branch_id(0)
			, node_id(0)
			, position()
		{
		}

		NodeRef(uint32_t branch, uint32_t node, Vec2 pos = Vec2())
			: branch_id(branch)
			, node_id(node)
			, position(pos)
		{
		}
	};

	struct Branch
	{
		std::vector<Node> nodes;
		uint32_t level;
		// Physics
		PhysicSegment segment;
		NodeRef root;

		Branch()
			: level(0)
		{}

		Branch(const Node& node, uint32_t lvl, const NodeRef& root_ref = NodeRef())
			: nodes{node}
			, level(lvl)
			, root(root_ref)
		{}

		void update(float dt)
		{
			segment.update(dt);
		}

		void translate(Vec2 v)
		{
			segment.translate(v);
			for (Node& n : nodes) {
				n.position += v;
			}
		}

		void translateTo(Vec2 position)
		{
			const Vec2 delta = position - root.position;
			root.position = position;
			translate(delta);
		}

		void initializePhysics()
		{
			segment = PhysicSegment(nodes.front().position, nodes.back().position);
			const float joint_strength(4000.0f * std::powf(0.4f, float(level)));
			segment.direction = segment.direction * joint_strength;
		}
	};

	struct Leaf
	{
		NodeRef attach;

		Particule free_particule;
		Particule broken_part;

		Vec2 target_direction;
		Vec2 acceleration;

		sf::Color color;
		float cut_threshold;
		float size;

		Leaf(NodeRef anchor, const Vec2& dir)
			: attach(anchor)
			, free_particule(anchor.position + dir)
			, target_direction(dir * RNGf::getRange(1.0f, 4.0f))
			, cut_threshold(0.4f + RNGf::getUnder(1.0f))
			, size(1.0f)
		{
			color = sf::Color(255, static_cast<uint8_t>(168 + RNGf::getRange(80.0f)), 0);
			
		}

		void solveAttach()
		{
			const float target_length = 1.0f;
			Vec2 delta = free_particule.position - attach.position;
			const float length = delta.normalize();
			const float dist_delta = target_length - length;
			/*if (std::abs(dist_delta) > cut_threshold) {
				cut();
			}*/

			free_particule.move(delta * dist_delta);
		}

		void solveLink()
		{
			const float length = 1.0f;
			const Vec2 delta = free_particule.position - broken_part.position;
			const float dist_delta = 1.0f - delta.getLength();
			free_particule.move(delta.getNormalized() * (0.5f * dist_delta));
			broken_part.move(delta.getNormalized() * (-0.5f * dist_delta));
		}

		Vec2 getDir() const
		{
			return free_particule.position - attach.position;
		}

		Vec2 getPosition() const
		{
			return attach.position;
		}

		void moveTo(Vec2 position)
		{
			const Vec2 delta = position - attach.position;
			attach.position = position;
			free_particule.position += delta;
			free_particule.old_position += delta;
		}

		/*void cut()
		{
			broken_part = Particule(attach->pos);
			attach = nullptr;
			target_direction = Vec2(0.0f, 1.0f);
		}*/

		void applyWind(const Wind& wind)
		{
			const float ratio = 1.0f;
			const float wind_force = wind.strength * (wind.speed ? ratio : 1.0f);
			free_particule.acceleration += Vec2(1.0f, RNGf::getRange(2.0f)) * wind_force;
		}

		void update(float dt)
		{
			solveAttach();
			free_particule.update(dt);
			// Reset acceleration
			free_particule.acceleration = target_direction;
			/*broken_part.update(dt);
			broken_part.acceleration = target_direction * joint_strenght;*/
		}
	};

	struct Tree
	{
		std::vector<Branch> branches;
		std::vector<Leaf> leaves;

		Tree() = default;

		void updateBranches(float dt)
		{
			for (Branch& b : branches) {
				b.update(dt);
			}
		}

		void updateLeaves(float dt)
		{
			for (Leaf& l : leaves) {
				l.update(dt);
			}
		}

		void updateStructure()
		{
			// Apply resulting translations
			rotateBranches();
			translateBranches();
			translateLeaves();
		}

		void update(float dt)
		{
			// Branch physics
			updateBranches(dt);
			// Leaves physics
			updateLeaves(dt);
			// Apply resulting transformations
			updateStructure();
		}

		void applyWind(const std::vector<Wind>& wind)
		{
			for (const Wind& w : wind) {
				for (Leaf& l : leaves) {
					w.apply(l.free_particule);
				}

				for (Branch& b : branches) {
					w.apply(b.segment.moving_point);
				}
			}
		}

		void rotateBranches()
		{
			for (Branch& b : branches) {
				rotateBranchTarget(b);
			}
		}

		void rotateBranchTarget(Branch& b)
		{
			const RotMat2 mat(b.segment.delta_angle);
			const Vec2 origin = b.nodes.front().position;
			for (Node& n : b.nodes) {
				n.position.rotate(origin, mat);
			}
		}

		void translateBranches()
		{
			uint64_t branches_count = branches.size();
			for (uint64_t i(1); i < branches_count; ++i) {
				Branch& b = branches[i];
				b.translateTo(getNode(b.root).position);
			}
		}

		Node& getNode(const NodeRef& ref)
		{
			return branches[ref.branch_id].nodes[ref.node_id];
		}

		void translateLeaves()
		{
			for (Leaf& l : leaves) {
				l.moveTo(getNode(l.attach).position);
			}
		}

		uint64_t getNodesCount() const
		{
			uint64_t res = 0;
			for (const Branch& b : branches) {
				res += b.nodes.size();
			}
			return res;
		}

		void generateSkeleton()
		{
			for (Branch& b : branches) {
				b.initializePhysics();
			}
		}
	};
}
