#pragma once

#include "PhysicsObject.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API ConceptObject : public PhysicsObject
	{
	public:
		ConceptObject();
		virtual ~ConceptObject();

		virtual void ApplyForcesAndTorques(Simulation* sim, double currentTime);
	};
}