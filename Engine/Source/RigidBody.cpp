#include "RigidBody.h"
#include "AxisAlignedBoundingBox.h"
#include "NumericalIntegrator.hpp"

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
			Matrix3x3 partialInertiaTensor;
			partialInertiaTensor.ele[0][0] = r.y * r.y + r.z * r.z;
			partialInertiaTensor.ele[0][1] = -r.x * r.y;
			partialInertiaTensor.ele[0][2] = r.x * r.z;
			partialInertiaTensor.ele[1][0] = -r.y * r.x;
			partialInertiaTensor.ele[1][1] = r.x * r.x + r.z * r.z;
			partialInertiaTensor.ele[1][2] = -r.y * r.z;
			partialInertiaTensor.ele[2][0] = -r.z * r.x;
			partialInertiaTensor.ele[2][1] = -r.z * r.y;
			partialInertiaTensor.ele[2][2] = r.x * r.x + r.y * r.y;
			this->bodySpaceInertiaTensor += partialInertiaTensor * particleMass;
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

/*virtual*/ void RigidBody::PrepareForTick()
{
	this->netForce = Vector3(0.0, 0.0, 0.0);
	this->netTorque = Vector3(0.0, 0.0, 0.0);
}

/*virtual*/ void RigidBody::Tick(double deltaTime)
{
	EulerIntegrator<Vector3> vectorValuedIntegrator(deltaTime);

	vectorValuedIntegrator.Integrate(this->linearMomentum, this->linearMomentum, 0.0, deltaTime, [this](double currentTime, const Vector3& currentValue, Vector3& currentValueDerivative) {
		currentValueDerivative = this->netForce;
	});

	vectorValuedIntegrator.Integrate(this->position, this->position, 0.0, deltaTime, [this](double currentTime, const Vector3& currentValue, Vector3& currentValueDerivative) {
		Vector3 velocity = this->linearMomentum / this->mass;
		currentValueDerivative = velocity;
	});

	vectorValuedIntegrator.Integrate(this->angularMomentum, this->angularMomentum, 0.0, deltaTime, [this](double currentTime, const Vector3& currentValue, Vector3& currentValueDerivative) {
		currentValueDerivative = this->netTorque;
	});

	EulerIntegrator<Matrix3x3> matrixValuedIntegrator(deltaTime);
	*matrixValuedIntegrator.stabilizeFunc = [](Matrix3x3& currentValue) {
		currentValue.Orthonormalize();
	};

	matrixValuedIntegrator.Integrate(this->orientation, this->orientation, 0.0, deltaTime, [this](double currentTime, const Matrix3x3& currentValue, Matrix3x3& currentValueDerivative) {
		Matrix3x3 orientationInv(this->orientation);
		orientationInv.Transpose();

		Matrix3x3 inertiaTensorInv = this->orientation * this->bodySpaceInertiaTensorInv * orientationInv;
		
		Vector3 angularVelocity = inertiaTensorInv * this->angularMomentum;
		Matrix3x3 angularVelocityMatrix;
		angularVelocityMatrix.SetForCrossProduct(angularVelocity);

		currentValueDerivative = angularVelocityMatrix * currentValue;
	});
}