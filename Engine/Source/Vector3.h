#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API Vector3
	{
	public:
		Vector3();
		Vector3(const Vector3& vector);
		Vector3(double x, double y, double z);
		virtual ~Vector3();

		double Length() const;
		double InnerProduct(const Vector3& vector) const;
		Vector3 CrossProduct(const Vector3& vector) const;
		bool Normalize(double* length = nullptr);
		Vector3 Normalized() const;
		void Decompose(const Vector3& unitVector, Vector3& projection, Vector3& rejection) const;
		void Rotate(const Vector3& axis, double angle);
		double AngleBetween(const Vector3& vector) const;

		void operator=(const Vector3& vector);
		void operator+=(const Vector3& vector);
		void operator-=(const Vector3& vector);
		void operator*=(double scalar);
		Vector3 operator-() const;

		double x, y, z;
	};

	PHYSICS_ENGINE_API Vector3 operator+(const Vector3& vectorA, const Vector3& vectorB);
	PHYSICS_ENGINE_API Vector3 operator-(const Vector3& vectorA, const Vector3& vectorB);
	PHYSICS_ENGINE_API Vector3 operator*(const Vector3& vector, double scalar);
	PHYSICS_ENGINE_API Vector3 operator*(double scalar, const Vector3& vector);
	PHYSICS_ENGINE_API Vector3 operator/(const Vector3& vector, double scalar);
}