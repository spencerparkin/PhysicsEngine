#pragma once

#include "PhysicsObject.h"

namespace PhysicsEngine
{
	class Vector3;
	class VectorN;

	class PHYSICS_ENGINE_API PhysicalObject : public PhysicsObject
	{
	public:
		PhysicalObject();
		virtual ~PhysicalObject();

		virtual double GetMass() const = 0;
		virtual void AccumulateForce(const Vector3& force) = 0;
		virtual void AccumulateTorque(const Vector3& torque);
		virtual int GetStateSpaceRequirement() const;
		virtual void SetStateFromVector(const VectorN& stateVector, int& i);
		virtual void GetStateToVector(VectorN& stateVector, int& i) const;
		virtual void CalcStateDerivatives(VectorN& stateVectorDerivative, int& i) const;
		virtual void ZeroNetForcesAndTorques();
	};
}