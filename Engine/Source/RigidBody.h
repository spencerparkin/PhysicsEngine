#pragma once

#include "PhysicalObject.h"
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
	class PHYSICS_ENGINE_API RigidBody : public PhysicalObject
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
		void SetLocation(const Vector3& location) { this->position = location; }
		void SetOrientation(const Matrix3x3& orientation) { this->orientation = orientation; }
		Vector3 GetVelocity() const;
		void SetVelocity(const Vector3& velocity);
		Vector3 GetAngularVelocity() const;
		void SetAngularVelocity(const Vector3& angularVelocity);

		virtual void ZeroNetForcesAndTorques() override;
		virtual double GetMass() const override;
		virtual Vector3 GetCenter() const override;
		virtual void SetCenter(const Vector3& center) override;
		virtual void AccumulateForce(const Vector3& force) override;
		virtual void AccumulateTorque(const Vector3& torque) override;
		virtual int GetStateSpaceRequirement() const override;
		virtual void SetStateFromVector(const VectorN& stateVector, int& i) override;
		virtual void GetStateToVector(VectorN& stateVector, int& i) const override;
		virtual void CalcStateDerivatives(VectorN& stateVectorDerivative, int& i) const override;
		virtual CollisionResult IsInCollsionWith(const PhysicalObject* physicalObject) const override;
		virtual CollisionResolution ResolveCollisionWith(PhysicalObject* physicalObject) override;
		virtual bool GetBoundingBox(AxisAlignedBoundingBox& boundingBox) const override;

	private:

		enum class ContactPointClassification
		{
			UNKNOWN,
			FACE_INTERIOR,
			EDGE_INTERIOR,
			VERTEX
		};

		ContactPointClassification ClassifyContactPoint(const Vector3& contactPoint, const PolygonMesh& mesh, Vector3& normal, Vector3& tangent);

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