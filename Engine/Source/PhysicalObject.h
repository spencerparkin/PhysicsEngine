#pragma once

#include "PhysicsObject.h"

namespace PhysicsEngine
{
	class Vector3;
	class VectorN;
	class AxisAlignedBoundingBox;

	class PHYSICS_ENGINE_API PhysicalObject : public PhysicsObject
	{
	public:
		PhysicalObject();
		virtual ~PhysicalObject();

		enum class CollisionResult
		{
			TRY_OTHER_WAY,
			IN_COLLISION,
			NOT_IN_COLLISION
		};

		enum class CollisionResolution
		{
			TRY_OTHER_WAY,
			FAILED,
			SUCCEEDED
		};

		virtual double GetMass() const = 0;
		virtual void AccumulateForce(const Vector3& force) = 0;
		virtual void AccumulateTorque(const Vector3& torque);
		virtual void ZeroNetForcesAndTorques();
		virtual void Integrate(double deltaTime);
		virtual CollisionResult IsInCollsionWith(const PhysicalObject* physicalObject) const;
		virtual CollisionResolution ResolveCollisionWith(const PhysicalObject* physicalObject);
		virtual bool GetBoundingBox(AxisAlignedBoundingBox& boundingBox) const;
	};
}