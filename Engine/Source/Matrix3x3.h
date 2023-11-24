#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class Vector3;
	class Quaternion;

	class PHYSICS_ENGINE_API Matrix3x3
	{
	public:
		Matrix3x3();
		Matrix3x3(const Matrix3x3& matrix);
		Matrix3x3(const Vector3& xAxis, const Vector3& yAxis, const Vector3& zAxis);
		virtual ~Matrix3x3();

		double Determinant() const;
		void SetIdentity();
		void SetAxes(const Vector3& xAxis, const Vector3& yAxis, const Vector3& zAxis);
		void GetAxes(Vector3& xAxis, Vector3& yAxis, Vector3& zAxis) const;
		void SetRotation(const Vector3& axis, double angle);
		void GetRotation(Vector3& axis, double angle) const;
		void SetRotation(const Quaternion& quat);
		void GetRotation(Quaternion& quat) const;
		void OuterProduct(const Vector3& vectorA, const Vector3& vectorB);
		bool Invert();
		void Transpose();
		bool Orthonormalize();
		Vector3 TransformPoint(const Vector3& point) const;

		void operator=(const Matrix3x3& matrix);
		void operator+=(const Matrix3x3& matrix);
		void operator-=(const Matrix3x3& matrix);
		void operator*=(double scalar);

		double ele[3][3];
	};

	PHYSICS_ENGINE_API Matrix3x3 operator+(const Matrix3x3& matA, const Matrix3x3& matB);
	PHYSICS_ENGINE_API Matrix3x3 operator-(const Matrix3x3& matA, const Matrix3x3& matB);
	PHYSICS_ENGINE_API Matrix3x3 operator*(const Matrix3x3& matA, const Matrix3x3& matB);
	PHYSICS_ENGINE_API Matrix3x3 operator/(const Matrix3x3& matA, const Matrix3x3& matB);
	PHYSICS_ENGINE_API Matrix3x3 operator*(const Matrix3x3& mat, double scalar);
	PHYSICS_ENGINE_API Matrix3x3 operator*(double scalar, const Matrix3x3& mat);
}