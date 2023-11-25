#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	// Examples of derivatives of this class include particles, springs, rigid-bodies, etc.
	class PHYSICS_ENGINE_API PhysicsObject
	{
	public:
		PhysicsObject();
		virtual ~PhysicsObject();

		virtual void DeleteSelf();		// May need to override this so that object is freed in correct heap.
		virtual void PrepareForTick();
		virtual void Tick(double deltaTime);

		//virtual void SolveConstraints(...);
		//virtual void GetBoundingBox(...);
	};
}