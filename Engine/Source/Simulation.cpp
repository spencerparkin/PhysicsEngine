#include "Simulation.h"
#include "PhysicsObject.h"

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
		pair.second->ApplyForces(this);

	for (auto pair : *this->physicsObjectMap)
		pair.second->Integrate(this, deltaTime);

	// TODO: Resolve constraints?
}

void Simulation::GetPhysicsObjectArray(std::vector<PhysicsObject*>& physicsObjectList)
{
	physicsObjectList.clear();
	for (auto pair : *this->physicsObjectMap)
		physicsObjectList.push_back(pair.second);
}

PhysicsObject* Simulation::FindPhysicsObject(const std::string& name)
{
	PhysicsObjectMap::iterator iter = this->physicsObjectMap->find(name);
	if (iter == this->physicsObjectMap->end())
		return nullptr;

	return iter->second;
}