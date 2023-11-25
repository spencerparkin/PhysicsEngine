#include "Gravity.h"
#include "RigidBody.h"
#include "Simulation.h"

using namespace PhysicsEngine;

Gravity::Gravity()
{
	this->direction = Vector3(0.0, -1.0, 0.0);
	this->acceleration = 9.8;
}

/*virtual*/ Gravity::~Gravity()
{
}

/*static*/ Gravity* Gravity::Create()
{
	return new Gravity();
}

/*virtual*/ void Gravity::ApplyForces(Simulation* sim)
{
	Simulation::PhysicsObjectMap& physicsObjectMap = sim->GetPhysicsObjectMap();
	for (auto pair : physicsObjectMap)
	{
		PhysicsObject* physicsObject = pair.second;
		RigidBody* rigidBody = dynamic_cast<RigidBody*>(physicsObject);
		if (rigidBody)
		{
			Vector3 gravityForce = rigidBody->GetMass() * this->direction * this->acceleration;
			rigidBody->AccumulateForce(gravityForce);
		}
	}
}