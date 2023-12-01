#pragma once

#include "Vector3.h"

namespace PhysicsEngine
{
	class Matrix3x3;

	class PHYSICS_ENGINE_API Plane
	{
	public:
		Plane();
		Plane(const Plane& plane);
		Plane(const Vector3& point, const Vector3& normal);
		virtual ~Plane();

		enum class Side
		{
			BACK,
			FRONT,
			NEITHER
		};

		double SignedDistanceToPoint(const Vector3& point) const;
		Vector3 CalcCenter() const;
		Side WhichSide(const Vector3& point, double tickness = PHY_ENG_SMALL_EPS) const;
		void Transform(const Vector3& translation, const Matrix3x3& rotation);
		Vector3 NearestPoint(const Vector3& point) const;

		Vector3 normal;
		double D;
	};
}