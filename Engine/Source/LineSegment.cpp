#include "LineSegment.h"
#include "Plane.h"

using namespace PhysicsEngine;

LineSegment::LineSegment()
{
}

LineSegment::LineSegment(const LineSegment& lineSegment)
{
	this->pointA = lineSegment.pointA;
	this->pointB = lineSegment.pointB;
}

LineSegment::LineSegment(const Vector3& pointA, const Vector3& pointB)
{
	this->pointA = pointA;
	this->pointB = pointB;
}

/*virtual*/ LineSegment::~LineSegment()
{
}

Vector3 LineSegment::Lerp(double alpha) const
{
	return this->pointA + alpha * (this->pointB - this->pointA);
}

/*static*/ bool LineSegment::Intersect(const LineSegment& lineSegA, const LineSegment& lineSegB, Vector3& intersectionPoint, double eps /*= PHY_ENG_SMALL_EPS*/)
{
	Vector3 numeratorVector = (lineSegB.pointB - lineSegB.pointA).CrossProduct(lineSegA.pointA - lineSegB.pointA);
	Vector3 denominatorVector = (lineSegB.pointB - lineSegB.pointA).CrossProduct(lineSegA.pointB - lineSegB.pointA);

	Vector3 numeratorVectorNormalized(numeratorVector);
	Vector3 denominatorVectorNormalized(denominatorVector);

	double numerator = 0.0;
	double denominator = 0.0;

	if (!numeratorVectorNormalized.Normalize(&numerator))
		return false;

	if (!denominatorVectorNormalized.Normalize(&denominator))
		return false;

	if (denominator == 0.0)
		return false;

	double scale = 1.0 / denominator;
	if (::isinf(scale) || ::isnan(scale))
		return false;

	// This is how we would check that the lines intersect at all, but it may be better
	// to do the epsilon bounds check against intersection points found on A and B as below.
#if 0
	// Verify that the 4 points are coplanar to within the given epsilon.
	Vector3 vector = numeratorVectorNormalized.CrossProduct(denominatorVectorNormalized);
	if (vector.Length() > eps)
		return false;
#endif

	// Make sure the lerp-alpha is approximately in the range [0,1].
	double alpha = -numerator / denominator;
	if (alpha < -eps || alpha > 1.0 + eps)
		return false;

	Vector3 intersectionPointA = lineSegA.Lerp(alpha);
	Vector3 intersectionPointB = lineSegB.NearestPoint(intersectionPointA);
	double distance = (intersectionPointA - intersectionPointB).Length();
	if (distance > eps)
		return false;

	intersectionPoint = (intersectionPointA + intersectionPointB) / 2.0;
	return true;
}

Vector3 LineSegment::NearestPoint(const Vector3& point) const
{
	Vector3 direction = this->pointB - this->pointA;
	direction.Normalize();
	Vector3 nearestPoint = this->pointA + (point - this->pointA).InnerProduct(direction) * direction;
	return nearestPoint;
}

bool LineSegment::IntersectWith(const Plane& plane, Vector3& intersectionPoint, double eps /*= PHY_ENG_SMALL_EPS*/) const
{
	double numerator = plane.D - this->pointA.InnerProduct(plane.normal);
	double denominator = (this->pointB - this->pointA).InnerProduct(plane.normal);
	if (denominator == 0.0)
		return false;

	double alpha = numerator / denominator;
	if (::isinf(alpha) || ::isnan(alpha))
		return false;

	if (alpha < -eps || alpha > 1.0 + eps)
		return false;

	intersectionPoint = this->Lerp(alpha);
	return true;
}

bool LineSegment::ContainsPoint(const Vector3& point, double thickness /*= PHY_ENG_SMALL_EPS*/) const
{
	Vector3 nearestPoint = this->NearestPoint(point);
	Vector3 difference = nearestPoint - point;
	double distanceSquared = difference.InnerProduct(difference);
	return distanceSquared <= thickness * thickness;
}