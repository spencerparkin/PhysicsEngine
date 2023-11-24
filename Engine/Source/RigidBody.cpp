#include "RigidBody.h"

using namespace PhysicsEngine;

RigidBody::RigidBody()
{
}

/*virtual*/ RigidBody::~RigidBody()
{
}

/*static*/ RigidBody* RigidBody::Create()
{
	return new RigidBody();
}

void RigidBody::MakeShape(const std::vector<Vector3>& pointArray, DensityFunc densityFunc)
{
	//...
}