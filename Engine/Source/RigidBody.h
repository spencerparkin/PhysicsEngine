#pragma once

#include "PhysicsObject.h"

namespace PhysicsEngine
{
	class Vector3;

	// Unlike a particle which is a single point-mass, here, a rigid-body represents
	// an orientable volume of mass with custom density.  Its shape will be defined
	// as a convex polygonal mesh.  This is all based on David Baraff's series of papers,
	// "An Introduction to Physically Based Modeling".
	class PHYSICS_ENGINE_API RigidBody : public PhysicsObject
	{
	public:
		RigidBody();
		virtual ~RigidBody();

		static RigidBody* Create();

		// TODO: How do you calculate the inertia tensor of a convex polyhedron?!
		//       One idea is to approximate the integral by iterating over a 3D matrix
		//       of voxels and just using those voxels that intersect the shape.

		typedef std::function<double(const Vector3&)> DensityFunc;

		// The shape made here is the convex hull of the given set of points.
		// It will be translated such that its center of mass is at the origin.
		void MakeShape(const std::vector<Vector3>& pointArray, DensityFunc densityFunc);

		// TODO: Own position, orientation, linear momentum, angular momentum.
	};
}