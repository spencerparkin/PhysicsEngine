#include "Simulation.h"
#include "PhysicsObject.h"
#include "PhysicalObject.h"
#include "NumericalIntegrator.hpp"
#include "VectorN.h"

using namespace PhysicsEngine;

Simulation::Simulation()
{
	this->physicsObjectMap = new PhysicsObjectMap();
	this->currentTime = 0.0;
	this->maxDeltaTime = 0.5;
}

/*virtual*/ Simulation::~Simulation()
{
	this->Clear();

	delete this->physicsObjectMap;
}

void Simulation::Clear()
{
	for (auto pair : *this->physicsObjectMap)
		pair.second->DeleteSelf();

	this->physicsObjectMap->clear();
}

bool Simulation::RemovePhysicsObject(const std::string& name)
{
	PhysicsObjectMap::iterator iter = this->physicsObjectMap->find(name);
	if (iter == this->physicsObjectMap->end())
		return false;

	iter->second->DeleteSelf();
	this->physicsObjectMap->erase(iter);
	return true;
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

	for (auto pair : *this->physicsObjectMap)
		pair.second->PrepareForTick(this);

	for (auto pair : *this->physicsObjectMap)
		pair.second->ApplyForcesAndTorques(this);

	// Gather all the physical objects in the simulation.
	std::vector<PhysicalObject*> physicalObjectArray;
	for (auto pair : *this->physicsObjectMap)
	{
		auto physicalObject = dynamic_cast<PhysicalObject*>(pair.second);
		if (physicalObject)
			physicalObjectArray.push_back(physicalObject);
	}

	// Determine how much space we need in our state vector.
	VectorN currentState;
	int N = 0;
	for (PhysicalObject* physicalObject : physicalObjectArray)
		N += physicalObject->GetStateSpaceRequirement();
	currentState.SetDimension(N);

	// Put the state of the entire system into the vector.
	int i = 0;
	for (PhysicalObject* physicalObject : physicalObjectArray)
		physicalObject->GetStateToVector(currentState, i);
	
	// Integrate the system over the time delta.
	VectorN newState(N);
	EulerIntegrator<VectorN> integrator(0.0025);	// TODO: Need to formalize the time-step parameter here.  For now, choose it arbitrarily.
	integrator.Integrate(currentState, newState, 0.0, deltaTime, [&physicalObjectArray](const VectorN& currentState, double currentTime, VectorN& currentStateDerivative) {
		currentStateDerivative.SetDimension(currentState.GetDimension());

		// Make sure all the physical objects know their current state at this step of integration before we calculate derivatives.
		int i = 0;
		for (PhysicalObject* physicalObject : physicalObjectArray)
			physicalObject->SetStateFromVector(currentState, i);

		// TODO: Check for collisions here and solve for constraints and collision forces?
		//       For contacting forces or rest positions, we may need to halt integration
		//       prematurely, calculate some new forces, then resume, or something like that.

		// Now ask the physical objects to calculate their derivatives.
		i = 0;
		for (PhysicalObject* physicalObject : physicalObjectArray)
			physicalObject->CalcStateDerivatives(currentStateDerivative, i);
	});

	// Finally, put the final state after integration back into the physical objects.
	i = 0;
	for (PhysicalObject* physicalObject : physicalObjectArray)
		physicalObject->SetStateFromVector(newState, i);
}

PhysicsObject* Simulation::FindPhysicsObject(const std::string& name)
{
	PhysicsObjectMap::iterator iter = this->physicsObjectMap->find(name);
	if (iter == this->physicsObjectMap->end())
		return nullptr;

	return iter->second;
}