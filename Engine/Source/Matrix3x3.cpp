#include "Matrix3x3.h"
#include "Vector3.h"
#include "Quaternion.h"

using namespace PhysicsEngine;

Matrix3x3::Matrix3x3()
{
	this->SetIdentity();
}

Matrix3x3::Matrix3x3(const Matrix3x3& matrix)
{
	*this = matrix;
}

Matrix3x3::Matrix3x3(const Vector3& xAxis, const Vector3& yAxis, const Vector3& zAxis)
{
	this->SetAxes(xAxis, yAxis, zAxis);
}

/*virtual*/ Matrix3x3::~Matrix3x3()
{
}

double Matrix3x3::Determinant() const
{
	return
		+ this->ele[0][0] * (this->ele[1][1] * this->ele[2][2] - this->ele[2][1] * this->ele[1][2])
		- this->ele[0][1] * (this->ele[1][0] * this->ele[2][2] - this->ele[2][0] * this->ele[1][2])
		+ this->ele[0][2] * (this->ele[1][0] * this->ele[2][1] - this->ele[2][0] * this->ele[1][1]);
}

void Matrix3x3::SetZero()
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] = 0.0;
}

void Matrix3x3::SetIdentity()
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] = (i == j) ? 1.0 : 0.0;
}

void Matrix3x3::SetAxes(const Vector3& xAxis, const Vector3& yAxis, const Vector3& zAxis)
{
	this->ele[0][0] = xAxis.x;
	this->ele[0][1] = xAxis.y;
	this->ele[0][2] = xAxis.z;

	this->ele[1][0] = yAxis.x;
	this->ele[1][1] = yAxis.y;
	this->ele[1][2] = yAxis.z;

	this->ele[2][0] = zAxis.x;
	this->ele[2][1] = zAxis.y;
	this->ele[2][2] = zAxis.z;
}

void Matrix3x3::GetAxes(Vector3& xAxis, Vector3& yAxis, Vector3& zAxis) const
{
	xAxis.x = this->ele[0][0];
	xAxis.y = this->ele[0][1];
	xAxis.z = this->ele[0][2];

	yAxis.x = this->ele[1][0];
	yAxis.y = this->ele[1][1];
	yAxis.z = this->ele[1][2];

	zAxis.x = this->ele[2][0];
	zAxis.y = this->ele[2][1];
	zAxis.z = this->ele[2][2];
}

void Matrix3x3::SetRotation(const Vector3& axis, double angle)
{
	Vector3 xAxis(1.0, 0.0, 0.0);
	Vector3 yAxis(0.0, 1.0, 0.0);
	Vector3 zAxis(0.0, 0.0, 1.0);

	xAxis.Rotate(axis, angle);
	yAxis.Rotate(axis, angle);
	zAxis.Rotate(axis, angle);

	this->SetAxes(xAxis, yAxis, zAxis);
}

void Matrix3x3::GetRotation(Vector3& axis, double angle) const
{
	Quaternion quat;
	this->GetRotation(quat);
	quat.GetRotation(axis, angle);
}

void Matrix3x3::SetRotation(const Quaternion& quat)
{
	Vector3 xAxis(1.0, 0.0, 0.0);
	Vector3 yAxis(0.0, 1.0, 0.0);
	Vector3 zAxis(0.0, 0.0, 1.0);

	xAxis = quat.TransformPoint(xAxis);
	yAxis = quat.TransformPoint(yAxis);
	zAxis = quat.TransformPoint(zAxis);

	this->SetAxes(xAxis, yAxis, zAxis);
}

void Matrix3x3::GetRotation(Quaternion& quat) const
{
	// This is Cayley's method taken from "A Survey on the Computation of Quaternions from Rotation Matrices" by Sarabandi & Thomas.

	double r11 = this->ele[0][0];
	double r21 = this->ele[1][0];
	double r31 = this->ele[2][0];

	double r12 = this->ele[0][1];
	double r22 = this->ele[1][1];
	double r32 = this->ele[2][1];

	double r13 = this->ele[0][2];
	double r23 = this->ele[1][2];
	double r33 = this->ele[2][2];

	quat.w = 0.25 * ::sqrt(PHY_ENG_SQUARED(r11 + r22 + r33 + 1.0) + PHY_ENG_SQUARED(r32 - r23) + PHY_ENG_SQUARED(r13 - r31) + PHY_ENG_SQUARED(r21 - r12));
	quat.x = 0.25 * ::sqrt(PHY_ENG_SQUARED(r32 - r23) + PHY_ENG_SQUARED(r11 - r22 - r33 + 1.0) + PHY_ENG_SQUARED(r21 + r12) + PHY_ENG_SQUARED(r31 + r13)) * PHY_ENG_SIGN(r32 - r23);
	quat.y = 0.25 * ::sqrt(PHY_ENG_SQUARED(r13 - r31) + PHY_ENG_SQUARED(r21 + r12) + PHY_ENG_SQUARED(r22 - r11 - r33 + 1.0) + PHY_ENG_SQUARED(r32 + r23)) * PHY_ENG_SIGN(r13 - r31);
	quat.z = 0.25 * ::sqrt(PHY_ENG_SQUARED(r21 - r12) + PHY_ENG_SQUARED(r31 + r13) + PHY_ENG_SQUARED(r32 + r23) + PHY_ENG_SQUARED(r33 - r11 - r22 + 1.0)) * PHY_ENG_SIGN(r21 - r12);
}

void Matrix3x3::OuterProduct(const Vector3& vectorA, const Vector3& vectorB)
{
	this->ele[0][0] = vectorA.x * vectorB.x;
	this->ele[0][1] = vectorA.y * vectorB.x;
	this->ele[0][2] = vectorA.z * vectorB.x;

	this->ele[1][0] = vectorA.x * vectorB.y;
	this->ele[1][1] = vectorA.y * vectorB.y;
	this->ele[1][2] = vectorA.z * vectorB.y;

	this->ele[2][0] = vectorA.x * vectorB.z;
	this->ele[2][1] = vectorA.y * vectorB.z;
	this->ele[2][2] = vectorA.z * vectorB.z;
}

void Matrix3x3::SetForCrossProduct(const Vector3& vector)
{
	this->ele[0][0] = 0.0;
	this->ele[0][1] = vector.z;
	this->ele[0][2] = -vector.y;

	this->ele[1][0] = -vector.z;
	this->ele[1][1] = 0.0;
	this->ele[1][2] = vector.x;

	this->ele[2][0] = vector.y;
	this->ele[2][1] = -vector.x;
	this->ele[2][2] = 0.0;
}

bool Matrix3x3::Invert()
{
	double det = this->Determinant();
	if (det == 0.0)
		return false;

	double scale = 1.0 / det;
	if (::isnan(scale) || ::isinf(scale))
		return false;

	Matrix3x3 matrix;

	matrix.ele[0][0] = this->ele[1][1] * this->ele[2][2] - this->ele[2][1] * this->ele[1][2];
	matrix.ele[0][1] = this->ele[0][2] * this->ele[2][1] - this->ele[2][2] * this->ele[0][1];
	matrix.ele[0][2] = this->ele[0][1] * this->ele[1][2] - this->ele[1][1] * this->ele[0][2];

	matrix.ele[1][0] = this->ele[1][2] * this->ele[2][0] - this->ele[2][2] * this->ele[1][0];
	matrix.ele[1][1] = this->ele[0][0] * this->ele[2][2] - this->ele[2][0] * this->ele[0][2];
	matrix.ele[1][2] = this->ele[0][2] * this->ele[1][0] - this->ele[1][2] * this->ele[0][0];

	matrix.ele[2][0] = this->ele[1][0] * this->ele[2][1] - this->ele[2][0] * this->ele[1][1];
	matrix.ele[2][1] = this->ele[0][1] * this->ele[2][0] - this->ele[2][1] * this->ele[0][0];
	matrix.ele[2][2] = this->ele[0][0] * this->ele[1][1] - this->ele[1][0] * this->ele[0][1];

	*this = matrix * scale;

	return true;
}

void Matrix3x3::Transpose()
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < i; j++)
			this->ele[i][j] = this->ele[j][i];
}

bool Matrix3x3::Orthonormalize()
{
	// TODO: Write this.
	return false;
}

Vector3 Matrix3x3::TransformPoint(const Vector3& point) const
{
	Vector3 result;

	result.x =
		this->ele[0][0] * point.x +
		this->ele[0][1] * point.y +
		this->ele[0][2] * point.z;

	result.y =
		this->ele[1][0] * point.x +
		this->ele[1][1] * point.y +
		this->ele[1][2] * point.z;

	result.z =
		this->ele[2][0] * point.x +
		this->ele[2][1] * point.y +
		this->ele[2][2] * point.z;

	return result;
}

void Matrix3x3::operator=(const Matrix3x3& matrix)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] = matrix.ele[i][j];
}

void Matrix3x3::operator+=(const Matrix3x3& matrix)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] += matrix.ele[i][j];
}

void Matrix3x3::operator-=(const Matrix3x3& matrix)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] -= matrix.ele[i][j];
}

void Matrix3x3::operator*=(double scalar)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] *= scalar;
}

namespace PhysicsEngine
{
	Matrix3x3 operator+(const Matrix3x3& matA, const Matrix3x3& matB)
	{
		Matrix3x3 result;

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.ele[i][j] = matA.ele[i][j] + matB.ele[i][j];

		return result;
	}

	Matrix3x3 operator-(const Matrix3x3& matA, const Matrix3x3& matB)
	{
		Matrix3x3 result;

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.ele[i][j] = matA.ele[i][j] - matB.ele[i][j];

		return result;
	}

	Matrix3x3 operator*(const Matrix3x3& matA, const Matrix3x3& matB)
	{
		Matrix3x3 result;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				result.ele[i][j] =
					matA.ele[i][0] * matB.ele[0][j] +
					matA.ele[i][1] * matB.ele[1][j] +
					matA.ele[i][2] * matB.ele[2][j];
			}
		}

		return result;
	}

	Matrix3x3 operator/(const Matrix3x3& matA, const Matrix3x3& matB)
	{
		Matrix3x3 result;
		Matrix3x3 matBInv(matB);
		if (matBInv.Invert())
			result = matA * matBInv;
		return result;
	}

	Matrix3x3 operator*(const Matrix3x3& mat, double scalar)
	{
		Matrix3x3 result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.ele[i][j] = mat.ele[i][j] * scalar;
		return result;
	}

	Matrix3x3 operator*(double scalar, const Matrix3x3& mat)
	{
		Matrix3x3 result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.ele[i][j] = mat.ele[i][j] * scalar;
		return result;
	}

	Vector3 operator*(const Matrix3x3& mat, const Vector3 vector)
	{
		return mat.TransformPoint(vector);
	}
}