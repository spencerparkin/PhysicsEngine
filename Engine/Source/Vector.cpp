#include "Vector.h"

using namespace PhysicsEngine;

Vector::Vector()
{
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

Vector::Vector(const Vector& vector)
{
	this->x = vector.x;
	this->y = vector.y;
	this->z = vector.z;
}

Vector::Vector(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

/*virtual*/ Vector::~Vector()
{
}

double Vector::Length() const
{
	return ::sqrt(this->Dot(*this));
}

double Vector::Dot(const Vector& vector) const
{
	return this->x * vector.x + this->y * vector.y + this->z * vector.z;
}

void Vector::Cross(const Vector& vectorA, const Vector& vectorB)
{
	this->x = vectorA.y * vectorB.z - vectorA.z * vectorB.y;
	this->y = vectorA.z * vectorB.x - vectorA.x * vectorB.z;
	this->z = vectorA.x * vectorB.y - vectorA.y * vectorB.x;
}

bool Vector::Normalize(double* length /*= nullptr*/)
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

void Vector::Decompose(const Vector& vector, Vector& projection, Vector& rejection) const
{
}

void Vector::Rotate(const Vector& vector, const Vector& axis, double angle)
{
}

void Vector::operator+=(const Vector& vector)
{
	this->x += vector.x;
	this->y += vector.y;
	this->z += vector.z;
}

void Vector::operator-=(const Vector& vector)
{
	this->x -= vector.x;
	this->y -= vector.y;
	this->z -= vector.z;
}

void Vector::operator*=(double scalar)
{
	this->x *= scalar;
	this->y *= scalar;
	this->z *= scalar;
}

namespace PhysicsEngine
{
	Vector operator+(const Vector& vectorA, const Vector& vectorB)
	{
		Vector result;
		result.x = vectorA.x + vectorB.x;
		result.y = vectorA.y + vectorB.y;
		result.z = vectorA.z + vectorB.z;
		return result;
	}

	Vector operator-(const Vector& vectorA, const Vector& vectorB)
	{
		Vector result;
		result.x = vectorA.x - vectorB.x;
		result.y = vectorA.y - vectorB.y;
		result.z = vectorA.z - vectorB.z;
		return result;
	}

	Vector operator*(const Vector& vector, double scalar)
	{
		Vector result;
		result.x = vector.x * scalar;
		result.y = vector.y * scalar;
		result.z = vector.z * scalar;
		return result;
	}

	Vector operator*(double scalar, const Vector& vector)
	{
		Vector result;
		result.x = vector.x * scalar;
		result.y = vector.y * scalar;
		result.z = vector.z * scalar;
		return result;
	}
}