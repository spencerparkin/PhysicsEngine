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

	// Calculate the body-space inertial tensor.  (I'm still not quite sure what this is, exactly.)
	this->bodySpaceInertiaTensor.SetZero();
	box.VisitSubBoxes(deltaLength, [this, &densityFunc, &centerOfMass](const AxisAlignedBoundingBox& subBox) {
		Vector3 particleCenter = subBox.Center();
		if (this->mesh.ContainsPoint(particleCenter))
		{
			double particleMass = densityFunc(particleCenter) * subBox.Volume();
			Vector3 particleVector = particleCenter - centerOfMass;
			Matrix3x3 matrix;
			matrix.OuterProduct(particleVector, -particleVector);
			double dot = particleVector.InnerProduct(particleVector);
			matrix.ele[0][0] += dot;
			matrix.ele[1][1] += dot;
			matrix.ele[2][2] += dot;
			this->bodySpaceInertiaTensor += matrix * particleMass;
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

		Matrix3x3 inertiaTensor = this->orientation * this->bodySpaceInertiaTensor * orientationInv;
		Matrix3x3 inertiaTensorInv = this->orientation * this->bodySpaceInertiaTensorInv * orientationInv;
		
		Vector3 angularVelocity = inertiaTensorInv * this->angularMomentum;
		Matrix3x3 angularVelocityMatrix;
		angularVelocityMatrix.OuterProduct(angularVelocity, angularVelocity);

		currentValueDerivative = angularVelocityMatrix * currentValue;
	});
}