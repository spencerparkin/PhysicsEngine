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
		physicsObject->DeleteSelf();

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

int Simulation::GetNumPhysicsObjects()
{
	return (int)this->physicsObjectArray->size();
}

PhysicsObject* Simulation::GetPhysicsObject(int i)
{
	if (i < 0 || i >= (int)this->physicsObjectArray->size())
		return nullptr;

	return (*this->physicsObjectArray)[i];
}