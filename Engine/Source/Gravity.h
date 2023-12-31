#pragma once

#include "ConceptObject.h"
#include "Vector3.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API Gravity : public ConceptObject
	{
	public:
		Gravity();
		virtual ~Gravity();

		static Gravity* Create();

		virtual void ApplyForcesAndTorques(Simulation* sim, double currentTime) override;

		void SetDirection(const Vector3& direction) { this->direction = direction; }
		const Vector3& GetDirection() const { return this->direction; }

		void SetAcceleration(double acceleration) { this->acceleration = acceleration; }
		double GetAcceleration() const { return this->acceleration; }

	private:
		Vector3 direction;
		double acceleration;
	};
}