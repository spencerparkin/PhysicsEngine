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

/*virtual*/ int PhysicalObject::GetStateSpaceRequirement() const
{
	return 0;
}

/*virtual*/ void PhysicalObject::SetStateFromVector(const VectorN& stateVector, int& i)
{
}

/*virtual*/ void PhysicalObject::GetStateToVector(VectorN& stateVector, int& i) const
{
}

/*virtual*/ void PhysicalObject::CalcStateDerivatives(VectorN& stateVectorDerivative, int& i) const
{
}

/*virtual*/ void PhysicalObject::ZeroNetForcesAndTorques()
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