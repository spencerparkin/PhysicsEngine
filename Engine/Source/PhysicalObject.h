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
		virtual Vector3 GetCenter() const = 0;
		virtual void SetCenter(const Vector3& center) = 0;
		virtual void AccumulateForce(const Vector3& force) = 0;
		virtual void AccumulateTorque(const Vector3& torque);
		virtual int GetStateSpaceRequirement() const;
		virtual void SetStateFromVector(const VectorN& stateVector, int& i);
		virtual void GetStateToVector(VectorN& stateVector, int& i) const;
		virtual void CalcStateDerivatives(VectorN& stateVectorDerivative, int& i) const;
		virtual void ZeroNetForcesAndTorques();
		virtual CollisionResult IsInCollsionWith(const PhysicalObject* physicalObject) const;
		virtual CollisionResolution ResolveCollisionWith(PhysicalObject* physicalObject);
		virtual bool GetBoundingBox(AxisAlignedBoundingBox& boundingBox) const;
	};
}