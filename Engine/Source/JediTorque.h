#pragma once

#include "PhysicsObject.h"
#include "Vector3.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API JediTorque : public PhysicsObject
	{
	public:
		JediTorque();
		virtual ~JediTorque();

		static JediTorque* Create();

		virtual void ApplyForcesAndTorques(Simulation* sim) override;

		void SetTorque(const Vector3& torque) { this->torque = torque; }
		const Vector3& GetTorque() const { return this->torque; }

		void SetTarget(const std::string& target) { *this->target = target; }
		const std::string& GetTarget() const { return *this->target; }

	private:
		Vector3 torque;
		std::string* target;
	};
}