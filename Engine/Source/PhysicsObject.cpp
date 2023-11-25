#include "PhysicsObject.h"

using namespace PhysicsEngine;

PhysicsObject::PhysicsObject()
{
	this->name = new std::string();
}

/*virtual*/ PhysicsObject::~PhysicsObject()
{
	delete this->name;
}

/*virtual*/ void PhysicsObject::DeleteSelf()
{
	delete this;
}

/*virtual*/ void PhysicsObject::PrepareForTick(Simulation* sim)
{
}

/*virtual*/ void PhysicsObject::ApplyForces(Simulation* sim)
{
}

/*virtual*/ void PhysicsObject::Integrate(Simulation* sim, double deltaTime)
{
}