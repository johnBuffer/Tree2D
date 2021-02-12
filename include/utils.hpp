#pragma once

#include "vec2.hpp"
#include <cmath>
#include <iostream>
#include <sstream>


const float PI = 3.141592653f;


float getVec2Angle(const Vec2& v1, const Vec2& v2)
{
	const float dot = v1.x * v2.x + v1.y * v2.y;
	const float det = v1.x * v2.y - v1.y * v2.x;
	return atan2(det, dot);
}


float getRandRange(float width)
{
	constexpr int32_t max_value = 10000;
	constexpr int32_t half = max_value / 2;
	constexpr float div = 1.0f / float(max_value);
	const float val = (rand() % max_value - half) * div * width;
	return val;
}


float getRandUnder(float max)
{
	constexpr int32_t max_value = 10000;
	constexpr int32_t half = max_value / 2;
	constexpr float div = 1.0f / float(max_value);
	const float val = (rand() % max_value) * div * max;
	return val;
}


std::string toString(float f, bool truncate = false)
{
	std::stringstream sx;
	sx << f;

	if (truncate) {
		return sx.str().substr(0, 4);
	}
	return sx.str();
}


float getLength(const Vec2& v)
{
	return sqrt(v.x*v.x + v.y*v.y);
}


Vec2 rotate(const Vec2& v, float angle, const Vec2& origin = Vec2(0.0f, 0.0f))
{
	const Vec2 origin_centered = v - origin;
	const float new_x = origin_centered.x * cos(angle) - origin_centered.y * sin(angle) + origin.x;
	const float new_y = origin_centered.x * sin(angle) + origin_centered.y * cos(angle) + origin.y;

	return Vec2(new_x, new_y);
}

float sign(float a)
{
	return a < 0.0f ? -1.0f : 1.0f;
}
