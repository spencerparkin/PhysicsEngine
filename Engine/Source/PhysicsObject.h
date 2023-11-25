#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class Simulation;

	// Examples of derivatives of this class include particles, springs, rigid-bodies, etc.,
	// as well as various types of forces.  Anything the simulation tracks derives from this.
	class PHYSICS_ENGINE_API PhysicsObject
	{
		friend class Simulation;

	public:
		PhysicsObject();
		virtual ~PhysicsObject();

		virtual void DeleteSelf();		// May need to override this so that the object is freed in correct heap.
		virtual void PrepareForTick(Simulation* sim);
		virtual void ApplyForces(Simulation* sim);
		virtual void Integrate(Simulation* sim, double deltaTime);

		const std::string& GetName() const { return *this->name; }

	private:
		std::string* name;
	};
}