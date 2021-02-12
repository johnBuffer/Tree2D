#pragma once
#include <SFML/Graphics.hpp>
#include "tree_2.hpp"


class TreeRenderer
{
private:
	sf::RenderTarget& m_target;
	sf::Sprite leaf_sprite;
	sf::Texture texture;

public:
	TreeRenderer(sf::RenderTarget& target)
		: m_target(target)
	{
		texture.loadFromFile("../res/leaf.png");
		leaf_sprite.setTexture(texture);
		leaf_sprite.setOrigin(800, 1485);
		leaf_sprite.setScale(0.02f, 0.02f);
	}

	//static void generateRenderData(const Tree& tree, std::vector<sf::VertexArray>& branches_va)
	//{
	//	// Create branches
	//	branches_va.clear();
	//	for (const Branch& b : tree.branches) {
	//		branches_va.emplace_back(sf::TriangleStrip, b.nodes.size() * 2);
	//		sf::VertexArray& va = branches_va.back();
	//		uint64_t i(0);
	//		for (const Node::Ptr n : b.nodes) {
	//			const float width = 0.5f * n->width;
	//			const Vec2 n_vec = n->growth_direction.getNormal() * width;
	//			va[2 * i].position = sf::Vector2f(n->pos.x + n_vec.x, n->pos.y + n_vec.y);
	//			va[2 * i + 1].position = sf::Vector2f(n->pos.x - n_vec.x, n->pos.y - n_vec.y);
	//			++i;
	//		}
	//	}
	//}

	static void generateRenderData(const v2::Tree& tree, std::vector<sf::VertexArray>& branches_va, sf::VertexArray& leaves_va)
	{
		// Create branches
		branches_va.clear();
		for (const v2::Branch& b : tree.branches) {
			branches_va.emplace_back(sf::TriangleStrip, b.nodes.size() * 2);
			sf::VertexArray& va = branches_va.back();
			uint64_t i(0);
			for (const v2::Node& n : b.nodes) {
				const float width = 0.5f * n.width;
				const Vec2 n_vec = n.direction.getNormal() * width;
				va[2 * i].position = sf::Vector2f(n.position.x + n_vec.x, n.position.y + n_vec.y);
				va[2 * i + 1].position = sf::Vector2f(n.position.x - n_vec.x, n.position.y - n_vec.y);
				++i;
			}
		}

		uint64_t i(0);
		const float leaf_length = 30.0f;
		const float leaf_width = 30.0f;
		leaves_va.resize(4 * tree.leaves.size());
		for (const v2::Leaf& l : tree.leaves) {
			const Vec2 leaf_dir = l.getDir().getNormalized();
			const Vec2 dir = leaf_dir * leaf_length * l.size;
			const Vec2 nrm = leaf_dir.getNormal() * (0.5f * leaf_width* l.size);
			const Vec2 attach = l.getPosition();
			const Vec2 pt1 = attach + nrm;
			const Vec2 pt2 = attach + nrm + dir;
			const Vec2 pt3 = attach - nrm + dir;
			const Vec2 pt4 = attach - nrm;
			// Geometry
			leaves_va[4 * i + 0].position = sf::Vector2f(pt1.x, pt1.y);
			leaves_va[4 * i + 1].position = sf::Vector2f(pt2.x, pt2.y);
			leaves_va[4 * i + 2].position = sf::Vector2f(pt3.x, pt3.y);
			leaves_va[4 * i + 3].position = sf::Vector2f(pt4.x, pt4.y);
			// Texture
			leaves_va[4 * i + 0].texCoords = sf::Vector2f(0.0f, 0.0f);
			leaves_va[4 * i + 1].texCoords = sf::Vector2f(1024.0f, 0.0f);
			leaves_va[4 * i + 2].texCoords = sf::Vector2f(1024.0f, 1024.0f);
			leaves_va[4 * i + 3].texCoords = sf::Vector2f(0.0f, 1024.0f);
			// Color
			leaves_va[4 * i + 0].color = l.color;
			leaves_va[4 * i + 1].color = l.color;
			leaves_va[4 * i + 2].color = l.color;
			leaves_va[4 * i + 3].color = l.color;

			++i;
		}
	}
};
