#include "PhysicalObject.h"

using namespace PhysicsEngine;

PhysicalObject::PhysicalObject()
{
}

/*virtual*/ PhysicalObject::~PhysicalObject()
{
}

/*virtual*/ void PhysicalObject::AccumulateTorque(const Vector3& torque)
{
}

/*virtual*/ void PhysicalObject::ZeroNetForcesAndTorques()
{
}

/*virtual*/ void PhysicalObject::Integrate(double deltaTime)
{
}

/*virtual*/ PhysicalObject::CollisionResult PhysicalObject::IsInCollsionWith(const PhysicalObject* physicalObject) const
{
	return CollisionResult::TRY_OTHER_WAY;
}

/*virtual*/ PhysicalObject::CollisionResolution PhysicalObject::ResolveCollisionWith(const PhysicalObject* physicalObject)
{
	return CollisionResolution::TRY_OTHER_WAY;
}

/*virtual*/ bool PhysicalObject::GetBoundingBox(AxisAlignedBoundingBox& boundingBox) const
{
	return false;
}