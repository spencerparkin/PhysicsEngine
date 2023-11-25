#include "PhysicsObject.h"

using namespace PhysicsEngine;

PhysicsObject::PhysicsObject()
{
}

/*virtual*/ PhysicsObject::~PhysicsObject()
{
}

/*virtual*/ void PhysicsObject::DeleteSelf()
{
	delete this;
}

/*virtual*/ void PhysicsObject::PrepareForTick()
{
}

/*virtual*/ void PhysicsObject::Tick(double deltaTime)
{
}