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
			moving_point.acceleration += direction;
			solveAttach();
			moving_point.update(dt);
			updateDeltaAngle();
		}
	};

	struct Node
	{
		Vec2 position;
		Vec2 last_position;
		Vec2 direction;
		uint32_t index;
		float length;
		float width;
		// Connected branch
		uint32_t branch_id;

		Node()
			: position()
			, last_position()
			, index(0)
			, width(1.0f)
			, branch_id(0)
		{}

		Node(const Vec2& pos, const Vec2& dir, uint32_t i, float l, float w, uint32_t connected_branch = 0)
			: position(pos)
			, last_position(pos)
			, direction(dir)
			, index(i)
			, length(l)
			, width(w)
			, branch_id(connected_branch)
		{}

		void resetPositionDelta()
		{
			last_position = position;
		}

		Vec2 getDelta() const
		{
			return position - last_position;
		}

		Vec2 getEnd() const
		{
			return position + direction * length;
		}
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

		NodeRef(uint32_t branch, uint32_t node, Vec2 pos)
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

		Branch()
			: level(0)
		{}

		Branch(const Node& node, uint32_t lvl)
			: nodes{node}
			, level(lvl)
		{}

		void update(float dt)
		{
			segment.update(dt);
		}

		void translate(const Vec2& v)
		{
			segment.translate(v);
			for (Node& n : nodes) {
				n.position += v;
			}
		}

		void finalizeUpdate()
		{
			for (Node& n : nodes) {
				n.resetPositionDelta();
			}
		}

		void initializePhysics()
		{
			segment = PhysicSegment(nodes.front().position, nodes.back().position);
			const float joint_strength(4000.0f * std::pow(0.4f, level));
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
		float joint_strenght;

		sf::Color color;
		float cut_threshold;
		float size;

		Leaf(NodeRef anchor, const Vec2& dir)
			: attach(anchor)
			, free_particule(anchor.position + dir)
			, target_direction(dir)
			, joint_strenght(RNGf::getRange(1.0f, 4.0f))
			, cut_threshold(0.4f + RNGf::getUnder(1.0f))
			, size(1.0f)
		{
			color = sf::Color(255, static_cast<uint8_t>(168 + RNGf::getRange(80.0f)), 0);
		}

		void solveAttach()
		{
			const float length = 1.0f;
			const Vec2 delta = free_particule.position - attach.position;
			const float dist_delta = 1.0f - delta.getLength();
			/*if (std::abs(dist_delta) > cut_threshold) {
				cut();
			}*/

			free_particule.move(delta.getNormalized() * dist_delta);
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

		void translate(const Vec2& delta)
		{
			attach.position += delta;
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
			free_particule.acceleration = target_direction * joint_strenght;
			broken_part.update(dt);
			broken_part.acceleration = target_direction * joint_strenght;
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

		void updateRest()
		{
			for (Branch& b : branches) {
				rotateBranchTarget(b);
			}
			// Apply resulting translations
			translateBranch(branches.front(), Vec2());
			translateLeaves();
			// Finalize update
			for (Branch& b : branches) {
				b.finalizeUpdate();
			}
		}

		void update(float dt)
		{
			// Branch physics
			updateBranches(dt);
			// Leaves physics
			updateLeaves(dt);
			// Apply resulting transformations
			updateRest();
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

		void rotateBranchTarget(Branch& b)
		{
			const RotMat2 mat(b.segment.delta_angle);
			const Vec2 origin = b.nodes.front().position;
			for (Node& n : b.nodes) {
				n.position.rotate(origin, mat);
			}
		}

		void translateBranch(Branch& b, const Vec2& v)
		{
			b.translate(v);
			for (Node& n : b.nodes) {
				if (n.branch_id) {
					const Vec2 delta = n.getDelta();
					translateBranch(branches[n.branch_id], delta);
				}
			}
		}

		Node& getNode(const NodeRef& ref)
		{
			return branches[ref.branch_id].nodes[ref.node_id];
		}

		void translateLeaves()
		{
			for (Leaf& l : leaves) {
				const Vec2 delta = getNode(l.attach).getDelta();
				l.translate(delta);
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

		void addLeaves()
		{
			uint32_t branch_id = 0;
			for (const Branch& b : branches) {
				const uint64_t nodes_count = b.nodes.size() - 1;
				const uint64_t leafs_count = 10;
				int32_t node_id = nodes_count;
				for (uint64_t i(0); i < leafs_count; ++i) {
					node_id -= i;
					if (node_id < 0) {
						break;
					}
					const float angle = RNGf::getRange(2.0f * PI);
					const NodeRef anchor(branch_id, node_id, b.nodes[node_id].position);
					leaves.emplace_back(anchor, Vec2(cos(angle), sin(angle)));
					leaves.back().size = 1.0f + (0.5f * i / float(leafs_count));
				}
				++branch_id;
			}
		}
	};
}
