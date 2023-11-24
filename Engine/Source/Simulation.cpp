#include "Simulation.h"
#include "PhysicsObject.h"

using namespace PhysicsEngine;

Simulation::Simulation()
{
	this->physicsObjectArray = new std::vector<PhysicsObject*>();
}

/*virtual*/ Simulation::~Simulation()
{
	this->Clear();

	delete this->physicsObjectArray;
}

void Simulation::Clear()
{
	for (PhysicsObject* physicsObject : *this->physicsObjectArray)
		delete physicsObject;

	this->physicsObjectArray->clear();
}

void Simulation::Tick(double deltaTime)
{
	for (PhysicsObject* physicsObject : *this->physicsObjectArray)
		physicsObject->PrepareForTick();

	// TODO: Accumulate forces and torque.

	for (PhysicsObject* physicsObject : *this->physicsObjectArray)
		physicsObject->Tick(deltaTime);

	// TODO: Resolve constraints?
}