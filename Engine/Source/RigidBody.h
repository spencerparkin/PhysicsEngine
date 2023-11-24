#pragma once

#include "PhysicsObject.h"

namespace PhysicsEngine
{
	// Unlike a particle which is a single point-mass, here, a rigid-body represents
	// an orientable volume of mass with uniform density.  Its shape will be defined
	// as a convex polygonal mesh.  This is all based on David Baraff's series of papers,
	// "An Introduction to Physically Based Modeling".
	class PHYSICS_ENGINE_API RigidBody : public PhysicsObject
	{
	public:
		RigidBody();
		virtual ~RigidBody();

		static RigidBody* Create();

		// TODO: How do you calculate the inertia tensor of a convex polyhedron?!
		//       For now, I think I'll just code up for the special case of a cube.

		// TODO: Own position, orientation, linear momentum, angular momentum.
	};
}