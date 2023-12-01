#include "GroundPlane.h"
#include "AxisAlignedBoundingBox.h"
#include "RigidBody.h"
#include "PolygonMesh.h"
#include "Matrix3x3.h"

using namespace PhysicsEngine;

GroundPlane::GroundPlane()
{
}

/*virtual*/ GroundPlane::~GroundPlane()
{
}

/*static*/ GroundPlane* GroundPlane::Create()
{
	return new GroundPlane();
}

/*virtual*/ Vector3 GroundPlane::GetCenter() const
{
	return this->plane.CalcCenter();
}

/*virtual*/ void GroundPlane::SetCenter(const Vector3& center)
{
	this->plane = Plane(center, plane.normal);
}

/*virtual*/ PhysicalObject::CollisionResult GroundPlane::IsInCollsionWith(const PhysicalObject* physicalObject) const
{
	auto rigidBody = dynamic_cast<const RigidBody*>(physicalObject);
	if (!rigidBody)
		return CollisionResult::TRY_OTHER_WAY;

	const PolygonMesh& mesh = rigidBody->GetMesh();
	for (const Vector3& vertex : mesh.GetVertexArray())
		if (this->plane.WhichSide(vertex, 0.0) != Plane::Side::FRONT)
			return CollisionResult::IN_COLLISION;

	return CollisionResult::NOT_IN_COLLISION;
}

/*virtual*/ PhysicalObject::CollisionResolution GroundPlane::ResolveCollisionWith(PhysicalObject* physicalObject)
{
	auto rigidBody = dynamic_cast<RigidBody*>(physicalObject);
	if (!rigidBody)
		return CollisionResolution::TRY_OTHER_WAY;

	const Vector3& contactNormal = this->plane.normal;

	std::vector<Vector3> contactPointArray;
	const PolygonMesh& mesh = rigidBody->GetMesh();
	for (const Vector3& vertex : mesh.GetVertexArray())
		if (this->plane.WhichSide(vertex, 0.0) != Plane::Side::FRONT)
			contactPointArray.push_back(vertex);

	Matrix3x3 orientation(rigidBody->GetOrientation());
	Matrix3x3 orientationInv(orientation);
	orientationInv.Transpose();
	Matrix3x3 inertiaTensorInv = orientation * rigidBody->GetBodySpaceInertiaTensorInv() * orientationInv;
	double coeficientOfRestitution = 1.0;
	bool collisionOccurred = false;

	do
	{
		collisionOccurred = false;

		for (const Vector3& contactPoint : contactPointArray)
		{
			Vector3 velocity = rigidBody->GetVelocity();
			Vector3 angularVelocity = inertiaTensorInv * rigidBody->GetAngularMomentum();
			Vector3 particleVec = contactPoint - rigidBody->GetLocation();
			Vector3 particleVel = rigidBody->GetVelocity() + angularVelocity.CrossProduct(particleVec);

			double relativeVelocity = contactNormal.InnerProduct(particleVel);
			if (relativeVelocity < 0.0)
			{
				Vector3 impulse(contactNormal);

				double term1 = 1.0 / rigidBody->GetMass();
				double term2 = contactNormal.InnerProduct((inertiaTensorInv * particleVec.CrossProduct(contactNormal)).CrossProduct(particleVec));
				double mag = -((1.0 + coeficientOfRestitution) * relativeVelocity) / (term1 + term2);
				assert(mag >= 0.0);
				impulse *= mag;

				rigidBody->SetLinearMomentum(rigidBody->GetLinearMomentum() + impulse);
				rigidBody->SetAngularMomentum(rigidBody->GetAngularMomentum().CrossProduct(impulse));

				collisionOccurred = true;
			}
		}
	} while (collisionOccurred);

	return CollisionResolution::SUCCEEDED;
}

/*virtual*/ bool GroundPlane::GetBoundingBox(AxisAlignedBoundingBox& boundingBox) const
{
	Vector3 center = this->plane.CalcCenter();
	
	boundingBox.min = center;
	boundingBox.max = center;

	double bigNumber = 9999999.0;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				Vector3 farQuadrantPoint = Vector3(
					bigNumber * ((i == 0) ? -1.0 : 1.0),
					bigNumber * ((j == 0) ? -1.0 : 1.0),
					bigNumber * ((k == 0) ? -1.0 : 1.0));

				Vector3 planePoint = this->plane.NearestPoint(farQuadrantPoint);
				boundingBox.ExpandToIncludePoint(planePoint);
			}
		}
	}

	boundingBox.Fatten(PHY_ENG_OBESE_EPS);
	return true;
}