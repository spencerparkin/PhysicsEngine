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

		virtual double GetMass() const;
		virtual Vector3 GetCenter() const;
		virtual void SetCenter(const Vector3& center);
		virtual void AccumulateForce(const Vector3& force);
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