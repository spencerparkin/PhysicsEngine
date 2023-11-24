#include "Quaternion.h"
#include "Vector3.h"

using namespace PhysicsEngine;

Quaternion::Quaternion()
{
	this->SetIdentity();
}

Quaternion::Quaternion(const Quaternion& quat)
{
	this->w = quat.w;
	this->x = quat.x;
	this->y = quat.y;
	this->z = quat.z;
}

Quaternion::Quaternion(double w, double x, double y, double z)
{
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}

/*virtual*/ Quaternion::~Quaternion()
{
}

double Quaternion::SquareMagnitude() const
{
	return
		this->w * this->w +
		this->x * this->x +
		this->y * this->y +
		this->z * this->z;
}

double Quaternion::Magnitude() const
{
	return ::sqrt(this->SquareMagnitude());
}

bool Quaternion::Normalize()
{
	double mag = this->Magnitude();
	if (mag == 0.0)
		return false;

	double scale = 1.0 / mag;
	if (::isnan(scale) || ::isinf(scale))
		return false;

	*this *= scale;
	return true;
}

void Quaternion::SetIdentity()
{
	this->w = 1.0;
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

void Quaternion::SetRotation(const Vector3& axis, double angle)
{
	//...
}

void Quaternion::GetRotation(Vector3& axis, double angle) const
{
	//...
}

void Quaternion::SetPoint(const Vector3& point)
{
	this->w = 0.0;
	this->x = point.x;
	this->y = point.y;
	this->z = point.z;
}

Vector3 Quaternion::GetPoint() const
{
	Vector3 result;
	result.x = this->x;
	result.y = this->y;
	result.z = this->z;
	return result;
}

Vector3 Quaternion::TransformPoint(const Vector3& point) const
{
	Quaternion result;

	Quaternion quatPoint;
	quatPoint.SetPoint(point);

	Quaternion quatInv(*this);
	if (quatInv.Invert())
		result = *this * quatPoint * quatInv;
		
	return result.GetPoint();
}

Vector3 Quaternion::RotatePoint(const Vector3& point) const
{
	Quaternion result;

	Quaternion quatPoint;
	quatPoint.SetPoint(point);

	Quaternion quatInv(*this);
	quatInv.Conjugate();
	result = *this * quatPoint * quatInv;

	return result.GetPoint();
}

bool Quaternion::Invert()
{
	double squareMag = this->SquareMagnitude();
	if (squareMag == 0.0)
		return false;

	double scale = 1.0 / squareMag;
	if (::isnan(scale) || isinf(scale))
		return false;

	this->Conjugate();
	*this *= scale;
	return true;
}

void Quaternion::Conjugate()
{
	this->x = -this->x;
	this->y = -this->y;
	this->z = -this->z;
}

void Quaternion::operator=(const Quaternion& quat)
{
	this->w = quat.w;
	this->x = quat.x;
	this->y = quat.y;
	this->z = quat.z;
}

void Quaternion::operator+=(const Quaternion& quat)
{
	this->w += quat.w;
	this->x += quat.x;
	this->y += quat.y;
	this->z += quat.z;
}

void Quaternion::operator-=(const Quaternion& quat)
{
	this->w -= quat.w;
	this->x -= quat.x;
	this->y -= quat.y;
	this->z -= quat.z;
}

void Quaternion::operator*=(const Quaternion& quat)
{
	Quaternion result = *this * quat;
	*this = result;
}

void Quaternion::operator*=(double scalar)
{
	this->w *= scalar;
	this->x *= scalar;
	this->y *= scalar;
	this->z *= scalar;
}

namespace PhysicsEngine
{
	Quaternion operator+(const Quaternion& quatA, const Quaternion& quatB)
	{
		Quaternion result;
		result.w = quatA.w + quatB.w;
		result.x = quatA.x + quatB.x;
		result.y = quatA.y + quatB.y;
		result.z = quatA.z + quatB.z;
		return result;
	}

	Quaternion operator-(const Quaternion& quatA, const Quaternion& quatB)
	{
		Quaternion result;
		result.w = quatA.w - quatB.w;
		result.x = quatA.x - quatB.x;
		result.y = quatA.y - quatB.y;
		result.z = quatA.z - quatB.z;
		return result;
	}

	Quaternion operator*(const Quaternion& quatA, const Quaternion& quatB)
	{
		Quaternion result;

		result.w =
			quatA.w * quatB.w -
			quatA.x * quatB.x -
			quatA.y * quatB.y -
			quatA.z * quatB.z;

		result.x =
			quatA.w * quatB.x +
			quatA.x * quatB.w +
			quatA.y * quatB.z -
			quatA.z * quatB.y;

		result.y =
			quatA.w * quatB.y -
			quatA.x * quatB.w +
			quatA.y * quatB.x +
			quatA.z * quatB.z;

		result.z =
			quatA.w * quatB.z +
			quatA.x * quatB.y -
			quatA.y * quatB.x +
			quatA.z * quatB.w;

		return result;
	}

	Quaternion operator/(const Quaternion& quatA, const Quaternion& quatB)
	{
		Quaternion result;
		Quaternion quatBInv(quatB);
		if (quatBInv.Invert())
			result = quatA * quatBInv;
		return result;
	}

	Quaternion operator*(const Quaternion& quat, double scalar)
	{
		Quaternion result;
		result.w = quat.w * scalar;
		result.x = quat.x * scalar;
		result.y = quat.y * scalar;
		result.z = quat.z * scalar;
		return result;
	}

	Quaternion operator*(double scalar, const Quaternion& quat)
	{
		Quaternion result;
		result.w = quat.w * scalar;
		result.x = quat.x * scalar;
		result.y = quat.y * scalar;
		result.z = quat.z * scalar;
		return result;
	}
}