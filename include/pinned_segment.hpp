#pragma once
#include "vec2.hpp"


struct Particule
{
	Vec2 position;
	Vec2 old_position;
	Vec2 acceleration;
	float mass;
	float inv_mass;

	Particule()
		: position()
		, old_position()
		, acceleration()
		, mass(1.0f)
		, inv_mass(1.0f)
	{}

	Particule(Vec2 pos, float m = 1.0f)
		: position(pos)
		, old_position(pos)
		, acceleration(0.0f, 0.0f)
		, mass(m)
		, inv_mass(1.0f / m)
	{}

	void update(float dt, float air_friction = 0.5f)
	{
		const Vec2 velocity = position - old_position;
		acceleration -= velocity * air_friction;
		const Vec2 new_pos = position + (velocity + acceleration * dt);
		old_position = position;
		position = new_pos;
		acceleration = Vec2(0.0f, 0.0f);
	}

	void move(Vec2 v)
	{
		position += v;
	}

	void applyForce(Vec2 v)
	{
		acceleration += v * inv_mass;
	}
};
