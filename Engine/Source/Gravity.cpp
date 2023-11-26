#include "Gravity.h"
#include "PhysicalObject.h"
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

/*virtual*/ void Gravity::ApplyForcesAndTorques(Simulation* sim, double currentTime)
{
	// Gravity is just universally applied to all physical objects of the simulation.
	Simulation::PhysicsObjectMap& physicsObjectMap = sim->GetPhysicsObjectMap();
	for (auto pair : physicsObjectMap)
	{
		PhysicalObject* physicalObject = dynamic_cast<PhysicalObject*>(pair.second);
		if (physicalObject)
		{
			Vector3 gravityForce = physicalObject->GetMass() * this->direction * this->acceleration;
			physicalObject->AccumulateForce(gravityForce);
		}
	}
}