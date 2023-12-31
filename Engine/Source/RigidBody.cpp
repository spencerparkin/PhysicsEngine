#include "RigidBody.h"
#include "AxisAlignedBoundingBox.h"
#include "Plane.h"
#include "VectorN.h"
#include "LineSegment.h"

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

	Vector3 angularVelocity = this->GetAngularVelocity();
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

Vector3 RigidBody::GetVelocity() const
{
	return this->linearMomentum / this->mass;
}

void RigidBody::SetVelocity(const Vector3& velocity)
{
	this->linearMomentum = velocity * this->mass;
}

Vector3 RigidBody::GetAngularVelocity() const
{
	Matrix3x3 orientationInv(this->orientation);
	orientationInv.Transpose();
	Matrix3x3 inertiaTensorInv = this->orientation * this->bodySpaceInertiaTensorInv * orientationInv;
	Vector3 angularVelocity = inertiaTensorInv * this->angularMomentum;
	return angularVelocity;
}

void RigidBody::SetAngularVelocity(const Vector3& angularVelocity)
{
	Matrix3x3 orientationInv(this->orientation);
	orientationInv.Transpose();
	Matrix3x3 inertiaTensor = this->orientation * this->bodySpaceInertiaTensor * orientationInv;
	this->angularMomentum = inertiaTensor * angularVelocity;
}

/*virtual*/ double RigidBody::GetMass() const
{
	return this->mass;
}

/*virtual*/ Vector3 RigidBody::GetCenter() const
{
	return this->GetLocation();
}

/*virtual*/ void RigidBody::SetCenter(const Vector3& center)
{
	this->SetLocation(center);
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
	const RigidBody* rigidBody = dynamic_cast<const RigidBody*>(physicalObject);
	if (!rigidBody)
		return CollisionResult::TRY_OTHER_WAY;

	auto meshA = std::shared_ptr<PolygonMesh>(this->mesh.Clone());
	auto meshB = std::shared_ptr<PolygonMesh>(rigidBody->mesh.Clone());

	meshA->Transform(this->position, this->orientation);
	meshB->Transform(rigidBody->position, rigidBody->orientation);

	if (PolygonMesh::ConvexHullsIntersect(*meshA, *meshB))
		return CollisionResult::IN_COLLISION;

	return CollisionResult::NOT_IN_COLLISION;
}

/*virtual*/ bool RigidBody::GetBoundingBox(AxisAlignedBoundingBox& boundingBox) const
{
	const std::vector<Vector3>& vertexArray = this->mesh.GetVertexArray();
	if (vertexArray.size() == 0)
		return false;

	boundingBox.min = this->position;
	boundingBox.max = this->position;

	for (const Vector3& localVertex : vertexArray)
	{
		Vector3 worldVertex = this->position + this->orientation * localVertex;
		boundingBox.ExpandToIncludePoint(worldVertex);
	}
	
	return true;
}

/*virtual*/ PhysicalObject::CollisionResolution RigidBody::ResolveCollisionWith(PhysicalObject* physicalObject)
{
	RigidBody* rigidBody = dynamic_cast<RigidBody*>(physicalObject);
	if (!rigidBody)
		return CollisionResolution::TRY_OTHER_WAY;

	RigidBody* bodyA = this;
	RigidBody* bodyB = rigidBody;

	auto meshA = std::shared_ptr<PolygonMesh>(bodyA->mesh.Clone());
	auto meshB = std::shared_ptr<PolygonMesh>(bodyB->mesh.Clone());

	meshA->Transform(this->position, this->orientation);
	meshB->Transform(rigidBody->position, rigidBody->orientation);

	std::vector<Vector3> contactPointArray;
	PolygonMesh::ConvexHullsIntersect(*meshA, *meshB, &contactPointArray);

	std::vector<Vector3> contactNormalArray;

	for (const Vector3& contactPoint : contactPointArray)
	{
		Vector3 normalA, tangentA;
		Vector3 normalB, tangentB;

		ContactPointClassification classificationA = this->ClassifyContactPoint(contactPoint, *meshA, normalA, tangentA);	
		ContactPointClassification classificationB = this->ClassifyContactPoint(contactPoint, *meshB, normalB, tangentB);
		
		if (classificationA == ContactPointClassification::UNKNOWN || classificationB == ContactPointClassification::UNKNOWN)
			return CollisionResolution::FAILED;

		Vector3 contactNormal;

		switch (classificationA)
		{
			case ContactPointClassification::VERTEX:
			{
				switch (classificationB)
				{
				case ContactPointClassification::VERTEX:
					contactNormal = normalB - normalA;	// Not so sure about this, but it is a rare case.
					contactNormal.Normalize();
					break;
				case ContactPointClassification::EDGE_INTERIOR:
				case ContactPointClassification::FACE_INTERIOR:
					contactNormal = normalB;
					break;
				}
				break;
			}
			case ContactPointClassification::EDGE_INTERIOR:
			{
				switch (classificationB)
				{
				case ContactPointClassification::VERTEX:
					contactNormal = normalA;
					break;
				case ContactPointClassification::EDGE_INTERIOR:
					contactNormal = tangentA.CrossProduct(tangentB);
					contactNormal.Normalize();
					break;
				case ContactPointClassification::FACE_INTERIOR:
					return CollisionResolution::FAILED;
				}
				break;
			}
			case ContactPointClassification::FACE_INTERIOR:
			{
				switch (classificationB)
				{
				case ContactPointClassification::VERTEX:
					contactNormal = normalA;
					break;
				case ContactPointClassification::EDGE_INTERIOR:
				case ContactPointClassification::FACE_INTERIOR:
					return CollisionResolution::FAILED;
				}
				break;
			}
		}

		contactNormalArray.push_back(contactNormal);
	}

	assert(contactNormalArray.size() == contactPointArray.size());

	// Lastly, before we can use the contact normals, we need them to all face a consistent direction.
	// By convention, we'll have them all face away from the given rigid body.  Our method here isn't
	// fool-proof, but will it work well enough?
	Vector3 center = rigidBody->position;
	for (int i = 0; i < (signed)contactPointArray.size(); i++)
	{
		const Vector3& point = contactPointArray[i];
		Vector3& normal = contactNormalArray[i];
		Vector3 vectorA = (point + normal * PHY_ENG_OBESE_EPS) - center;
		Vector3 vectorB = (point - normal * PHY_ENG_OBESE_EPS) - center;
		double squareDistanceA = vectorA.InnerProduct(vectorA);
		double squareDistanceB = vectorB.InnerProduct(vectorB);
		if (squareDistanceA < squareDistanceB)
			normal = -normal;
	}

	// If they are not touching one another, there is nothing for us to do.
	// This shouldn't happen since we shouldn't be called unless the calling
	// code is sure that we're in collision with the given object.
	if (contactPointArray.size() == 0)
		return CollisionResolution::FAILED;

	Matrix3x3 orientationInvA(bodyA->orientation);
	Matrix3x3 orientationInvB(bodyB->orientation);

	orientationInvA.Transpose();
	orientationInvB.Transpose();

	Matrix3x3 inertiaTensorInvA = bodyA->orientation * bodyA->bodySpaceInertiaTensorInv * orientationInvA;
	Matrix3x3 inertiaTensorInvB = bodyB->orientation * bodyB->bodySpaceInertiaTensorInv * orientationInvB;

	bool collisionOccurred = false;
	do
	{
		collisionOccurred = false;

		for (int i = 0; i < (signed)contactPointArray.size(); i++)
		{
			const Vector3& contactPoint = contactPointArray[i];
			const Vector3& contactNormal = contactNormalArray[i];

			Vector3 bodyVelocityA = bodyA->GetVelocity();
			Vector3 bodyVelocityB = bodyB->GetVelocity();

			Vector3 bodyAngularVelocityA = inertiaTensorInvA * bodyA->angularMomentum;
			Vector3 bodyAngularVelocityB = inertiaTensorInvB * bodyB->angularMomentum;

			Vector3 particleVecA = contactPoint - bodyA->GetLocation();
			Vector3 particleVecB = contactPoint - bodyB->GetLocation();

			Vector3 particleVelA = bodyVelocityA + bodyAngularVelocityA.CrossProduct(particleVecA);
			Vector3 particleVelB = bodyVelocityB + bodyAngularVelocityB.CrossProduct(particleVecB);

			double relativeVelocity = contactNormal.InnerProduct(particleVelA - particleVelB);
			if (relativeVelocity < 0.0)
			{
				// We have colliding contact in progress.  Resolve it.
				Vector3 impulse(contactNormal);

				double term1 = 1.0 / bodyA->mass;
				double term2 = 1.0 / bodyB->mass;
				double term3 = contactNormal.InnerProduct((inertiaTensorInvA * particleVecA.CrossProduct(contactNormal)).CrossProduct(particleVecA));
				double term4 = contactNormal.InnerProduct((inertiaTensorInvB * particleVecB.CrossProduct(contactNormal)).CrossProduct(particleVecB));

				double coeficientOfRestitution = 1.0;
				double mag = -((1.0 + coeficientOfRestitution) * relativeVelocity) / (term1 + term2 + term3 + term4);
				assert(mag >= 0.0);
				impulse *= mag;

				bodyA->linearMomentum += impulse;
				bodyB->linearMomentum -= impulse;
				bodyA->angularMomentum += particleVecA.CrossProduct(impulse);	// Apply the "impulsive torque" to A.
				bodyB->angularMomentum -= particleVecB.CrossProduct(impulse);	// Apply the "impulsive torque" to B.

				collisionOccurred = true;
			}
			else if (relativeVelocity == 0.0)
			{
				// TODO: Resting contact?!?!?!?????????

				return CollisionResolution::FAILED;
			}
			else
			{
				// In this case, the bodys are in contact, but moving away from one another.
				// There is nothing for us to resolve here.
			}
		}
	} while (collisionOccurred);

	return CollisionResolution::SUCCEEDED;
}

RigidBody::ContactPointClassification RigidBody::ClassifyContactPoint(const Vector3& contactPoint, const PolygonMesh& mesh, Vector3& normal, Vector3& tangent)
{
	for (int i = 0; i < (signed)mesh.GetVertexArray().size(); i++)
	{
		const Vector3& vertex = mesh.GetVertexArray()[i];
		double distance = (vertex - contactPoint).Length();
		if (distance < PHY_ENG_OBESE_EPS)
		{
			std::vector<const PolygonMesh::Polygon*> polygonArray;
			mesh.FindAllFacesSharingVertex(i, polygonArray);
			normal = Vector3(0.0, 0.0, 0.0);
			for (const PolygonMesh::Polygon* polygon : polygonArray)
			{
				Plane plane;
				polygon->MakePlane(plane);
				normal += plane.normal;
			}
			normal.Normalize();
			tangent = Vector3(0.0, 0.0, 0.0);
			return ContactPointClassification::VERTEX;
		}
	}

	std::vector<PolygonMesh::Edge> edgeArray;
	mesh.GenerateEdgeArray(edgeArray);
	for (const PolygonMesh::Edge& edge : edgeArray)
	{
		LineSegment edgeSegment;
		edgeSegment.pointA = mesh.GetVertexArray()[edge.i];
		edgeSegment.pointB = mesh.GetVertexArray()[edge.j];
		if (edgeSegment.ContainsPoint(contactPoint, PHY_ENG_OBESE_EPS))
		{
			std::vector<const PolygonMesh::Polygon*> polygonArray;
			mesh.FindAllFacesSharingEdge(edge, polygonArray);
			normal = Vector3(0.0, 0.0, 0.0);
			for (const PolygonMesh::Polygon* polygon : polygonArray)
			{
				Plane plane;
				polygon->MakePlane(plane);
				normal += plane.normal;
			}
			normal.Normalize();
			tangent = edgeSegment.pointB - edgeSegment.pointA;
			return ContactPointClassification::EDGE_INTERIOR;
		}
	}

	for (int i = 0; i < (signed)mesh.GetNumPolygons(); i++)
	{
		const PolygonMesh::Polygon* polygon = mesh.GetPolygon(i);
		if (polygon->ContainsPoint(contactPoint, PHY_ENG_FAT_EPS))
		{
			Plane plane;
			polygon->MakePlane(plane);
			normal = plane.normal;
			tangent = Vector3(0.0, 0.0, 0.0);
			return ContactPointClassification::FACE_INTERIOR;
		}
	}

	return ContactPointClassification::UNKNOWN;
}