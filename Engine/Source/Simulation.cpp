#include "Simulation.h"
#include "PhysicsObject.h"

using namespace PhysicsEngine;

Simulation::Simulation()
{
	this->physicsObjectArray = new std::vector<PhysicsObject*>();
	this->currentTime = 0.0;
	this->maxDeltaTime = 0.5;
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

void Simulation::Tick()
{
	if (this->currentTime == 0.0)
	{
		// This is our first tick.  Just initialize time and bail.
		this->currentTime = double(::clock()) / double(CLOCKS_PER_SEC);
		return;
	}
	
	double presentTime = double(::clock()) / double(CLOCKS_PER_SEC);
	double deltaTime = presentTime - this->currentTime;
	this->currentTime = presentTime;
	if (deltaTime > this->maxDeltaTime)
	{
		// The idea here is to prevent a debugger break, followed by a resume of
		// the program from creating a very large time-step in the simulation.
		return;
	}

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