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


template<typename T>
std::string toString(T v, bool truncate = false)
{
	std::stringstream sx;
	sx << v;

	if (truncate) {
		return sx.str().substr(0, 4);
	}
	return sx.str();
}


float getLength(const Vec2& v)
{
	return sqrt(v.x*v.x + v.y*v.y);
}


float sign(float a)
{
	return a < 0.0f ? -1.0f : 1.0f;
}
