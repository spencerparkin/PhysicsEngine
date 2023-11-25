#pragma once

#include "PhysicsObject.h"
#include "PolygonMesh.h"
#include "Matrix3x3.h"
#include "Vector3.h"

namespace PhysicsEngine
{
	class Vector3;
	class VectorN;

	// Unlike a particle which is a single point-mass, here, a rigid-body represents
	// an orientable volume of mass with custom density distribution.  Its shape will
	// be defined as a convex polygonal mesh.  This is all based on David Baraff's
	// series of papers, "An Introduction to Physically Based Modeling".
	class PHYSICS_ENGINE_API RigidBody : public PhysicsObject
	{
	public:
		RigidBody();
		virtual ~RigidBody();

		static RigidBody* Create();

		typedef std::function<double(const Vector3&)> DensityFunc;

		// The shape made here is the convex hull of the given set of points.
		// It will be translated such that its center of mass is at the origin.
		bool MakeShape(const std::vector<Vector3>& pointArray, double deltaLength, DensityFunc densityFunc);

		const PolygonMesh& GetMesh() const { return this->mesh; }
		const Vector3& GetLocation() const { return this->position; }
		const Matrix3x3& GetOrientation() const { return this->orientation; }

		virtual void PrepareForTick() override;
		virtual void Tick(double deltaTime) override;

	private:

		bool StateToVectorN(VectorN& stateVector) const;
		bool StateFromVectorN(const VectorN& stateVector);

		double mass;
		PolygonMesh mesh;
		Matrix3x3 bodySpaceInertiaTensor;
		Matrix3x3 bodySpaceInertiaTensorInv;
		Vector3 position;
		Matrix3x3 orientation;
		Vector3 linearMomentum;
		Vector3 angularMomentum;
		Vector3 netForce;
		Vector3 netTorque;
	};
}