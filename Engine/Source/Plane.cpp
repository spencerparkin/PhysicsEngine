#include "Plane.h"
#include "Matrix3x3.h"

using namespace PhysicsEngine;

Plane::Plane()
{
	this->normal = Vector3(0.0, 1.0, 0.0);
	this->D = 0.0;
}

Plane::Plane(const Plane& plane)
{
	this->normal = plane.normal;
	this->D = plane.D;
}

Plane::Plane(const Vector3& point, const Vector3& normal)
{
	this->normal = normal;
	this->normal.Normalize();
	this->D = point.InnerProduct(this->normal);
}

/*virtual*/ Plane::~Plane()
{
}

double Plane::SignedDistanceToPoint(const Vector3& point) const
{
	return point.InnerProduct(this->normal) - this->D;
}

Vector3 Plane::CalcCenter() const
{
	return this->normal * this->D;
}

Plane::Side Plane::WhichSide(const Vector3& point, double tickness /*= PHY_ENG_SMALL_EPS*/) const
{
	double signedDistance = this->SignedDistanceToPoint(point);

	if (signedDistance < -tickness / 2.0)
		return Side::BACK;

	if (signedDistance > tickness / 2.0)
		return Side::FRONT;

	return Side::NEITHER;
}

void Plane::Transform(const Vector3& translation, const Matrix3x3& rotation)
{
	Vector3 center = rotation * this->CalcCenter() + translation;
	this->normal = rotation * this->normal;
	this->normal.Normalize();
	this->D = center.InnerProduct(this->normal);
}

Vector3 Plane::NearestPoint(const Vector3& point) const
{
	return point - this->normal * this->SignedDistanceToPoint(point);
}