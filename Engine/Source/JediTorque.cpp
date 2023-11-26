#include "JediTorque.h"
#include "Simulation.h"
#include "PhysicalObject.h"

using namespace PhysicsEngine;

JediTorque::JediTorque()
{
	this->target = new std::string();
	this->torque = Vector3(0.0, 0.0, 0.0);
}

/*virtual*/ JediTorque::~JediTorque()
{
	delete this->target;
}

/*static*/ JediTorque* JediTorque::Create()
{
	return new JediTorque();
}

/*virtual*/ void JediTorque::ApplyForcesAndTorques(Simulation* sim, double currentTime)
{
	PhysicsObject* physicsObject = sim->FindPhysicsObject(*this->target);
	if (physicsObject)
	{
		PhysicalObject* physicalObject = dynamic_cast<PhysicalObject*>(physicsObject);
		if (physicalObject)
		{
			physicalObject->AccumulateTorque(this->torque);
		}
	}
}