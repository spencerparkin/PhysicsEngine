#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class Simulation;

	// These are anything the simulation tracks.
	class PHYSICS_ENGINE_API PhysicsObject
	{
		friend class Simulation;

	public:
		PhysicsObject();
		virtual ~PhysicsObject();

		virtual void DeleteSelf();		// May need to override this so that the object is freed in correct heap.

		const std::string& GetName() const { return *this->name; }

	private:
		std::string* name;
	};
}