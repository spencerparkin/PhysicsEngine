#include "RigidBody.h"
#include "AxisAlignedBoundingBox.h"
#include "NumericalIntegrator.hpp"
#include "VectorN.h"

using namespace PhysicsEngine;

RigidBody::RigidBody()
{
	this->mass = 0.0;
}

/*virtual*/ RigidBody::~RigidBody()
{
}

/*static*/ RigidBody* RigidBody::Create()
{
	return new RigidBody();
}

bool RigidBody::MakeShape(const std::vector<Vector3>& pointArray, double deltaLength, DensityFunc densityFunc)
{
	if (!this->mesh.GenerateConvexHull(pointArray))
		return false;

	AxisAlignedBoundingBox box;
	if (!this->mesh.CalcBoundingBox(box))
		return false;

	// Calculate the center of mass.
	Vector3 totalMoments(0.0, 0.0, 0.0);
	this->mass = 0.0;
	box.VisitSubBoxes(deltaLength, [this, &densityFunc, &totalMoments](const AxisAlignedBoundingBox& subBox) {
		Vector3 particleCenter = subBox.Center();
		if (this->mesh.ContainsPoint(particleCenter))
		{
			double particleMass = densityFunc(particleCenter) * subBox.Volume();
			this->mass += particleMass;
			totalMoments += particleMass * particleCenter;
		}
	});
	Vector3 centerOfMass = totalMoments / this->mass;

	// Calculate the body-space inertial tensor.  Note that in finding the relationship between
	// angular velocity and angular momentum, we get L = int(r x (w x r) dm).  Here, r is the position
	// of a particle in the body, w is the angular momentum, dm is the mass of the particle, and
	// we're integrating over all the particles of the objects.  Doing some rearrangement, this equation
	// can be re-written as L = I*w, where I is a matrix (or rank 2 tensor.)  This matrix, however,
	// is not constant, because it depends on the orientation of the body.  Fortunately, it can be shown
	// that the world-space inertia tensor I_{world} = R*I_{body}*R^T, where R is the current orientation
	// of the body.
	this->bodySpaceInertiaTensor.SetZero();
	box.VisitSubBoxes(deltaLength, [this, &densityFunc, &centerOfMass](const AxisAlignedBoundingBox& subBox) {
		Vector3 particleCenter = subBox.Center();
		if (this->mesh.ContainsPoint(particleCenter))
		{
			double particleMass = densityFunc(particleCenter) * subBox.Volume();
			Vector3 r = particleCenter - centerOfMass;
			Matrix3x3 particleInertiaTensor;
			particleInertiaTensor.ele[0][0] = r.y * r.y + r.z * r.z;
			particleInertiaTensor.ele[0][1] = -r.x * r.y;
			particleInertiaTensor.ele[0][2] = r.x * r.z;
			particleInertiaTensor.ele[1][0] = -r.y * r.x;
			particleInertiaTensor.ele[1][1] = r.x * r.x + r.z * r.z;
			particleInertiaTensor.ele[1][2] = -r.y * r.z;
			particleInertiaTensor.ele[2][0] = -r.z * r.x;
			particleInertiaTensor.ele[2][1] = -r.z * r.y;
			particleInertiaTensor.ele[2][2] = r.x * r.x + r.y * r.y;
			this->bodySpaceInertiaTensor += particleInertiaTensor * particleMass;
		}
	});

	// Cache the inverse of the inertial tensor as well.
	this->bodySpaceInertiaTensorInv = this->bodySpaceInertiaTensor;
	if (!this->bodySpaceInertiaTensorInv.Invert())
		return false;

	// Shift the mesh so that the center of mass is at origin in body space.
	this->mesh.Translate(-centerOfMass);

	// Reset our state variables.
	this->position = Vector3(0.0, 0.0, 0.0);
	this->orientation.SetIdentity();
	this->linearMomentum = Vector3(0.0, 0.0, 0.0);
	this->angularMomentum = Vector3(0.0, 0.0, 0.0);

	return true;
}

/*virtual*/ void RigidBody::ZeroNetForcesAndTorques()
{
	this->netForce = Vector3(0.0, 0.0, 0.0);
	this->netTorque = Vector3(0.0, 0.0, 0.0);
}

/*virtual*/ int RigidBody::GetStateSpaceRequirement() const
{
	// 3 for position, 9 for orientation, 3 for linear momentum, and 3 for angular momentum.
	return 18;
}

/*virtual*/ void RigidBody::SetStateFromVector(const VectorN& stateVector, int& i)
{
	this->position.x = stateVector[i++];
	this->position.y = stateVector[i++];
	this->position.z = stateVector[i++];

	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			this->orientation.ele[r][c] = stateVector[i++];

	this->orientation.Orthonormalize();

	this->linearMomentum.x = stateVector[i++];
	this->linearMomentum.y = stateVector[i++];
	this->linearMomentum.z = stateVector[i++];

	this->angularMomentum.x = stateVector[i++];
	this->angularMomentum.y = stateVector[i++];
	this->angularMomentum.z = stateVector[i++];
}

/*virtual*/ void RigidBody::GetStateToVector(VectorN& stateVector, int& i) const
{
	stateVector[i++] = this->position.x;
	stateVector[i++] = this->position.y;
	stateVector[i++] = this->position.z;

	const_cast<RigidBody*>(this)->orientation.Orthonormalize();

	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			stateVector[i++] = this->orientation.ele[r][c];

	stateVector[i++] = this->linearMomentum.x;
	stateVector[i++] = this->linearMomentum.y;
	stateVector[i++] = this->linearMomentum.z;

	stateVector[i++] = this->angularMomentum.x;
	stateVector[i++] = this->angularMomentum.y;
	stateVector[i++] = this->angularMomentum.z;
}

/*virtual*/ void RigidBody::CalcStateDerivatives(VectorN& stateVectorDerivative, int& i) const
{
	Vector3 velocity = this->linearMomentum / this->mass;

	stateVectorDerivative[i++] = velocity.x;
	stateVectorDerivative[i++] = velocity.y;
	stateVectorDerivative[i++] = velocity.z;

	const_cast<RigidBody*>(this)->orientation.Orthonormalize();

	Matrix3x3 orientationInv(this->orientation);
	orientationInv.Transpose();

	Matrix3x3 inertiaTensorInv = this->orientation * this->bodySpaceInertiaTensorInv * orientationInv;

	Vector3 angularVelocity = inertiaTensorInv * this->angularMomentum;
	Matrix3x3 angularVelocityMatrix;
	angularVelocityMatrix.SetForCrossProduct(angularVelocity);

	Matrix3x3 orientationDerivative = angularVelocityMatrix * this->orientation;

	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			stateVectorDerivative[i++] = orientationDerivative.ele[r][c];

	stateVectorDerivative[i++] = this->netForce.x;
	stateVectorDerivative[i++] = this->netForce.y;
	stateVectorDerivative[i++] = this->netForce.z;

	stateVectorDerivative[i++] = this->netTorque.x;
	stateVectorDerivative[i++] = this->netTorque.y;
	stateVectorDerivative[i++] = this->netTorque.z;
}

/*virtual*/ double RigidBody::GetMass() const
{
	return this->mass;
}

/*virtual*/ void RigidBody::AccumulateForce(const Vector3& force)
{
	this->netForce += force;
}

/*virtual*/ void RigidBody::AccumulateTorque(const Vector3& torque)
{
	this->netTorque += torque;
}

/*virtual*/ PhysicalObject::CollisionResult RigidBody::IsInCollsionWith(const PhysicalObject* physicalObject) const
{
	// TODO: Write this.
	return PhysicalObject::CollisionResult::UNKNOWN;
}

/*virtual*/ bool RigidBody::GetBoundingBox(AxisAlignedBoundingBox& boundingBox) const
{
	// TODO: Write this.
	return false;
}