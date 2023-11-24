#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class Vector3;

	class PHYSICS_ENGINE_API Quaternion
	{
	public:
		Quaternion();
		Quaternion(const Quaternion& quat);
		Quaternion(double w, double x, double y, double z);
		virtual ~Quaternion();

		double SquareMagnitude() const;
		double Magnitude() const;
		bool Normalize();
		void SetIdentity();
		void SetRotation(const Vector3& axis, double angle);
		void GetRotation(Vector3& axis, double angle) const;
		void SetPoint(const Vector3& point);
		Vector3 GetPoint() const;
		Vector3 TransformPoint(const Vector3& point) const;
		Vector3 RotatePoint(const Vector3& point) const;
		bool Invert();
		void Conjugate();

		void operator=(const Quaternion& quat);
		void operator+=(const Quaternion& quat);
		void operator-=(const Quaternion& quat);
		void operator*=(const Quaternion& quat);
		void operator*=(double scalar);

		double w, x, y, z;
	};

	PHYSICS_ENGINE_API Quaternion operator+(const Quaternion& quatA, const Quaternion& quatB);
	PHYSICS_ENGINE_API Quaternion operator-(const Quaternion& quatA, const Quaternion& quatB);
	PHYSICS_ENGINE_API Quaternion operator*(const Quaternion& quatA, const Quaternion& quatB);
	PHYSICS_ENGINE_API Quaternion operator/(const Quaternion& quatA, const Quaternion& quatB);
	PHYSICS_ENGINE_API Quaternion operator*(const Quaternion& quat, double scalar);
	PHYSICS_ENGINE_API Quaternion operator*(double scalar, const Quaternion& quat);
}