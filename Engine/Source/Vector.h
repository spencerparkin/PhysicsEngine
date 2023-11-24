#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API Vector
	{
	public:
		Vector();
		Vector(const Vector& vector);
		Vector(double x, double y, double z);
		virtual ~Vector();

		double Length() const;
		double Dot(const Vector& vector) const;
		void Cross(const Vector& vectorA, const Vector& vectorB);
		bool Normalize(double* length = nullptr);
		void Decompose(const Vector& vector, Vector& projection, Vector& rejection) const;
		void Rotate(const Vector& vector, const Vector& axis, double angle);

		void operator+=(const Vector& vector);
		void operator-=(const Vector& vector);
		void operator*=(double scalar);

		double x, y, z;
	};

	Vector operator+(const Vector& vectorA, const Vector& vectorB);
	Vector operator-(const Vector& vectorA, const Vector& vectorB);
	Vector operator*(const Vector& vector, double scalar);
	Vector operator*(double scalar, const Vector& vector);
}