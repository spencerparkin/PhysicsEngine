#include "JediForce.h"
#include "Simulation.h"
#include "PhysicalObject.h"

using namespace PhysicsEngine;

JediForce::JediForce()
{
	this->target = new std::string();
	this->force = Vector3(0.0, 0.0, 0.0);
}

/*virtual*/ JediForce::~JediForce()
{
	delete this->target;
}

/*static*/ JediForce* JediForce::Create()
{
	return new JediForce();
}

/*virtual*/ void JediForce::ApplyForcesAndTorques(Simulation* sim, double currentTime)
{
	PhysicsObject* physicsObject = sim->FindPhysicsObject(*this->target);
	if (physicsObject)
	{
		PhysicalObject* physicalObject = dynamic_cast<PhysicalObject*>(physicsObject);
		if (physicalObject)
		{
			physicalObject->AccumulateForce(this->force);
		}
	}
}