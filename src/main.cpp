#include <iostream>
#include <SFML/Graphics.hpp>
#include <string>
#include <iostream>
#include <cmath>

#include "tree_renderer.hpp"
#include "wind.hpp"

#include "tree.hpp"
#include "tree_builder.hpp"


int main()
{
    constexpr uint32_t WinWidth = 1920;
	constexpr uint32_t WinHeight = 1080;

	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(WinWidth, WinHeight), "Tree", sf::Style::Default, settings);
	window.setFramerateLimit(60);

	// 9f0f269
	// Perf: 0.36 ms
	// b01e334
	// Perf: 0.25 ms

	v2::TreeConf tree_conf{
		80.0f, // branch_width
		0.95f, // branch_width_ratio
		0.75f, // split_width_ratio
		0.5f, // deviation
		PI * 0.25f, // split angle
		0.1f, // branch_split_var;
		40.0f, // branch_length;
		0.96f, // branch_length_ratio;
		0.5f, // branch_split_proba;
		0.0f, // double split
		Vec2(0.0f, -0.5f), // Attraction
		8
	};

	sf::Texture texture;
	texture.loadFromFile("../res/leaf.png");

	sf::Font font;
	font.loadFromFile("../res/font.ttf");
	sf::Text text_profiler;
	text_profiler.setFont(font);
	text_profiler.setFillColor(sf::Color::White);
	text_profiler.setCharacterSize(24);

	std::vector<sf::VertexArray> branches_va;
	sf::VertexArray leaves_va(sf::Quads);
	v2::Tree tree = v2::TreeBuilder::build(Vec2(WinWidth * 0.5f, WinHeight), tree_conf);

	float base_wind_force = 0.05f;
	float max_wind_force = 30.0f;
	float current_wind_force = 0.0f;

	const float wind_force = 1.0f;
	std::vector<Wind> wind{
		Wind(100.0f, 3.f * wind_force, 700.0f),
		Wind(300.0f, 2.f * wind_force, 1050.0f),
		Wind(400.0f, 3.f * wind_force, 1208.0f),
		Wind(500.0f, 4.f * wind_force, 1400.0f),
	};

	const float dt = 0.016f;

	float time_sum_leaves = 0.0f;
	float time_sum_branches = 0.0f;
	float time_sum_rest = 0.0f;
	float img_count = 1.0f;

	bool boosting = false;

	bool draw_branches = true;
	bool draw_leaves = true;
	bool draw_debug = false;
	bool draw_wind_debug = false;

	sf::Clock clock;
	while (window.isOpen())
	{
		const sf::Vector2i mouse_pos = sf::Mouse::getPosition(window);

        sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close();
			} else if (event.type == sf::Event::KeyReleased) {
				if (event.key.code == sf::Keyboard::Space) {
					tree = v2::TreeBuilder::build(Vec2(WinWidth * 0.5f, WinHeight), tree_conf);
				}
				else if (event.key.code == sf::Keyboard::B) {
					draw_branches = !draw_branches;
				}
				else if (event.key.code == sf::Keyboard::L) {
					draw_leaves = !draw_leaves;
				}
				else if (event.key.code == sf::Keyboard::D) {
					draw_debug = !draw_debug;
				}
				else if (event.key.code == sf::Keyboard::W) {
					draw_wind_debug = !draw_wind_debug;
				}
				else {
					boosting = false;
					wind[0].strength = base_wind_force;
				}
			}
			else if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Space) {
					
				}
				else if (event.key.code == sf::Keyboard::Escape) {
					window.close();
				}
				else if (event.key.code == sf::Keyboard::Up) {
					for (Wind& w : wind) {
						w.strength *= 1.2f;
					}
				}
				else if (event.key.code == sf::Keyboard::Down) {
					for (Wind& w : wind) {
						w.strength /= 1.2f;
					}
				}
				else if (event.key.code == sf::Keyboard::W) {
					base_wind_force = 1.0f;
				}
				else {
					boosting = true;
					if (current_wind_force < max_wind_force) {
						current_wind_force += 0.1f;
					}
					wind[0].strength = current_wind_force;
				}
			}
		}

		for (Wind& w : wind) {
			w.update(dt, WinWidth);
		}

		tree.applyWind(wind);

		if (boosting) {
			for (v2::Branch& b : tree.branches) {
				b.segment.moving_point.acceleration += Vec2(1.0f, 0.0f) * wind_force;
			}
		}

		sf::Clock profiler_clock;
		tree.updateBranches(dt);
		const float elapsed_b = static_cast<float>(profiler_clock.getElapsedTime().asMicroseconds());
		time_sum_branches += elapsed_b;

		profiler_clock.restart();
		tree.updateLeaves(dt);
		const float elapsed_l = static_cast<float>(profiler_clock.getElapsedTime().asMicroseconds());
		time_sum_leaves += elapsed_l;

		profiler_clock.restart();
		tree.updateStructure();
		const float elapsed_r = static_cast<float>(profiler_clock.getElapsedTime().asMicroseconds());
		time_sum_rest += elapsed_r;

		window.clear(sf::Color::Black);

		const float text_offset = 24.0f;
		float text_y = 10.0f;
		text_profiler.setString("Structure simulation    " + toString(int(time_sum_branches / img_count)) + " us");
		text_profiler.setPosition(10.0f, text_y);
		window.draw(text_profiler);
		text_y += text_offset;

		text_profiler.setString("Leaves simulation       " + toString(int(time_sum_leaves / img_count)) + " us");
		text_profiler.setPosition(10.0f, text_y);
		window.draw(text_profiler);
		text_y += text_offset;

		text_profiler.setString("Structure update        " + toString(int(time_sum_rest / img_count)) + " us");
		text_profiler.setPosition(10.0f, text_y);
		window.draw(text_profiler);
		text_y += 2.0f * text_offset;

		text_profiler.setString("Physic simulation time  " + toString(0.001f * ((time_sum_leaves + time_sum_branches + time_sum_rest) / img_count), true) + "ms");
		text_profiler.setPosition(10.0f, text_y);
		window.draw(text_profiler);

		TreeRenderer::generateRenderData(tree, branches_va, leaves_va);
		if (draw_branches) {
			for (const auto& va : branches_va) {
				window.draw(va);
			}
		}
		if (draw_leaves) {
			sf::RenderStates states;
			states.texture = &texture;
			window.draw(leaves_va, states);
		}

		if (draw_debug) {
			sf::VertexArray va_debug(sf::Lines, 2 * tree.branches.size());
			uint32_t i = 0;
			for (const v2::Branch& b : tree.branches) {
				va_debug[2 * i + 0].position = sf::Vector2f(b.segment.attach_point.x, b.segment.attach_point.y);
				va_debug[2 * i + 1].position = sf::Vector2f(b.segment.moving_point.position.x, b.segment.moving_point.position.y);
				va_debug[2 * i + 0].color = sf::Color::Red;
				va_debug[2 * i + 1].color = sf::Color::Red;
				++i;
			}
			window.draw(va_debug);

			i = 0;
			for (const v2::Branch& b : tree.branches) {
				const float joint_strength(4000.0f * std::powf(0.4f, float(b.level)));
				const float length = joint_strength * 0.03f;
				sf::Vector2f bot(b.segment.moving_point.position.x, b.segment.moving_point.position.y);
				const Vec2& dir = b.segment.direction.getNormalized();
				sf::Vector2f top(bot + length * sf::Vector2f(dir.x, dir.y));
				va_debug[2 * i + 0].position = bot;
				va_debug[2 * i + 1].position = top;
				va_debug[2 * i + 0].color = sf::Color::Green;
				va_debug[2 * i + 1].color = sf::Color::Green;
				++i;
			}
			window.draw(va_debug);
		}

		if (draw_wind_debug) {
			for (const Wind& w : wind) {
				sf::RectangleShape wind_debug(sf::Vector2f(w.width, WinHeight));
				wind_debug.setPosition(w.pos_x - w.width * 0.5f, 0.0f);
				wind_debug.setFillColor(sf::Color(255, 0, 0, 100));
				window.draw(wind_debug);
			}
		}

		img_count += 1.0f;

        window.display();
    }

    return 0;
}
