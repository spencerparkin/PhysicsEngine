#pragma once

#include "PhysicsObject.h"

namespace PhysicsEngine
{
	class Vector3;

	class PHYSICS_ENGINE_API PhysicalObject : public PhysicsObject
	{
	public:
		PhysicalObject();
		virtual ~PhysicalObject();

		virtual double GetMass() const = 0;
		virtual void AccumulateForce(const Vector3& force) = 0;
		virtual void AccumulateTorque(const Vector3& torque);
	};
}