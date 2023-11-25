#pragma once

#include "PhysicsObject.h"
#include "Vector3.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API JediForce : public PhysicsObject
	{
	public:
		JediForce();
		virtual ~JediForce();

		virtual void ApplyForces(Simulation* sim) override;

		void SetForce(const Vector3& force) { this->force = force; }
		const Vector3& GetForce() const { return this->force; }

		void SetTarget(const std::string& target) { *this->target = target; }
		const std::string& GetTarget() const { return *this->target; }

	private:
		Vector3 force;
		std::string* target;
	};
}