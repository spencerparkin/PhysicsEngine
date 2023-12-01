#include "Vector3.h"

using namespace PhysicsEngine;

Vector3::Vector3()
{
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

Vector3::Vector3(const Vector3& vector)
{
	this->x = vector.x;
	this->y = vector.y;
	this->z = vector.z;
}

Vector3::Vector3(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

/*virtual*/ Vector3::~Vector3()
{
}

double Vector3::Length() const
{
	return ::sqrt(this->InnerProduct(*this));
}

void Vector3::SetOrthogonalTo(const Vector3& vector)
{
	double ax = ::abs(vector.x);
	double ay = ::abs(vector.y);
	double az = ::abs(vector.z);

	if (ax < ay && ax < az)
	{
		this->x = 0.0;
		this->y = az;
		this->z = -ay;
	}
	else if (ay < ax && ay < az)
	{
		this->x = az;
		this->y = 0.0;
		this->z = -ax;
	}
	else //if(az < ax && az < ax)
	{
		this->x = ay;
		this->y = -ax;
		this->z = 0.0;
	}
}

double Vector3::InnerProduct(const Vector3& vector) const
{
	return this->x * vector.x + this->y * vector.y + this->z * vector.z;
}

Vector3 Vector3::CrossProduct(const Vector3& vector) const
{
	Vector3 result;
	result.x = this->y * vector.z - this->z * vector.y;
	result.y = this->z * vector.x - this->x * vector.z;
	result.z = this->x * vector.y - this->y * vector.x;
	return result;
}

bool Vector3::Normalize(double* length /*= nullptr*/)
{
	double magnitude = this->Length();
	if (length)
		*length = magnitude;

	if (magnitude == 0.0)
		return false;

	double scale = 1.0 / magnitude;
	if (::isnan(scale) || ::isinf(scale))
		return false;

	*this *= scale;
	return true;
}

Vector3 Vector3::Normalized() const
{
	Vector3 vector(*this);
	if (!vector.Normalize())
		vector = Vector3(0.0, 0.0, 0.0);
	return vector;
}

void Vector3::Decompose(const Vector3& unitVector, Vector3& projection, Vector3& rejection) const
{
	projection = this->InnerProduct(unitVector) * unitVector;
	rejection = *this - projection;
}

void Vector3::Rotate(const Vector3& axis, double angle)
{
	Vector3 zAxis = axis.Normalized();
	Vector3 projection, rejection;
	this->Decompose(zAxis, projection, rejection);
	Vector3 xAxis = rejection;
	double radius = 0.0;
	xAxis.Normalize(&radius);
	Vector3 yAxis = zAxis.CrossProduct(xAxis);
	rejection = radius * ::cos(angle) * xAxis + radius * ::sin(angle) * yAxis;
	*this = projection + rejection;
}

double Vector3::AngleBetween(const Vector3& vector) const
{
	return ::acos(this->Normalized().InnerProduct(vector.Normalized()));
}

void Vector3::operator=(const Vector3& vector)
{
	this->x = vector.x;
	this->y = vector.y;
	this->z = vector.z;
}

void Vector3::operator+=(const Vector3& vector)
{
	this->x += vector.x;
	this->y += vector.y;
	this->z += vector.z;
}

void Vector3::operator-=(const Vector3& vector)
{
	this->x -= vector.x;
	this->y -= vector.y;
	this->z -= vector.z;
}

void Vector3::operator*=(double scalar)
{
	this->x *= scalar;
	this->y *= scalar;
	this->z *= scalar;
}

Vector3 Vector3::operator-() const
{
	return Vector3(
		-this->x,
		-this->y,
		-this->z);
}

namespace PhysicsEngine
{
	Vector3 operator+(const Vector3& vectorA, const Vector3& vectorB)
	{
		Vector3 result;
		result.x = vectorA.x + vectorB.x;
		result.y = vectorA.y + vectorB.y;
		result.z = vectorA.z + vectorB.z;
		return result;
	}

	Vector3 operator-(const Vector3& vectorA, const Vector3& vectorB)
	{
		Vector3 result;
		result.x = vectorA.x - vectorB.x;
		result.y = vectorA.y - vectorB.y;
		result.z = vectorA.z - vectorB.z;
		return result;
	}

	Vector3 operator*(const Vector3& vector, double scalar)
	{
		Vector3 result;
		result.x = vector.x * scalar;
		result.y = vector.y * scalar;
		result.z = vector.z * scalar;
		return result;
	}

	Vector3 operator*(double scalar, const Vector3& vector)
	{
		Vector3 result;
		result.x = vector.x * scalar;
		result.y = vector.y * scalar;
		result.z = vector.z * scalar;
		return result;
	}

	Vector3 operator/(const Vector3& vector, double scalar)
	{
		Vector3 result;
		result.x = vector.x / scalar;
		result.y = vector.y / scalar;
		result.z = vector.z / scalar;
		return result;
	}
}