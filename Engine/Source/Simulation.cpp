#include "Simulation.h"
#include "PhysicsObject.h"

using namespace PhysicsEngine;

Simulation::Simulation()
{
	this->currentTime = 0.0;
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

void Simulation::Step(double deltaTime)
{
	//...
}