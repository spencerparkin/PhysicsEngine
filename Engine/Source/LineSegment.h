#pragma once

#include "Vector3.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API LineSegment
	{
	public:
		LineSegment();
		LineSegment(const LineSegment& lineSegment);
		LineSegment(const Vector3& pointA, const Vector3& pointB);
		virtual ~LineSegment();

		Vector3 Lerp(double alpha) const;
		static bool Intersect(const LineSegment& lineSegA, const LineSegment& lineSegB, Vector3& intersectionPoint, double eps = PHY_ENG_EPS);

		Vector3 pointA, pointB;
	};
}