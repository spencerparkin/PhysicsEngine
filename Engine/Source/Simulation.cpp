#include "Simulation.h"
#include "PhysicsObject.h"
#include "PhysicalObject.h"
#include "ConceptObject.h"
#include "AxisAlignedBoundingBox.h"
#include "VectorN.h"

using namespace PhysicsEngine;

Simulation::Simulation()
{
	this->physicsObjectMap = new PhysicsObjectMap();
	this->physicalObjectArray = new std::vector<PhysicalObject*>();
	this->conceptObjectArray = new std::vector<ConceptObject*>();
	this->currentTime = 0.0;
	this->maxDeltaTime = 0.5;
	this->maxTimeStepSize = 0.0025;
	this->collisionTimeTolerance = 0.0005;
}

/*virtual*/ Simulation::~Simulation()
{
	this->Clear();

	delete this->physicsObjectMap;
	delete this->physicalObjectArray;
	delete this->conceptObjectArray;
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
	if (deltaTime > this->maxDeltaTime)
	{
		// The idea here is to prevent a debugger break, followed by a resume of
		// the program, from creating a very large time-step in the simulation.
		this->currentTime = presentTime;
		return;
	}

	// Bucket sort into physical and conceptual objects.
	this->physicalObjectArray->clear();
	this->conceptObjectArray->clear();
	for (auto pair : *this->physicsObjectMap)
	{
		auto physicalObject = dynamic_cast<PhysicalObject*>(pair.second);
		if (physicalObject)
			this->physicalObjectArray->push_back(physicalObject);

		auto conceptObject = dynamic_cast<ConceptObject*>(pair.second);
		if (conceptObject)
			this->conceptObjectArray->push_back(conceptObject);
	}

	// Determine how much space we need in our state vector.
	VectorN currentState;
	int N = 0;
	for (PhysicalObject* physicalObject : *this->physicalObjectArray)
		N += physicalObject->GetStateSpaceRequirement();
	currentState.SetDimension(N);

	// We assume each tick begins with no two physical objects interpenetrating.
	// Note that a more sophisticated system would abstract the notion of what
	// kind of integration technique we're using here, but I'm just going to use
	// Euler integration here for the sake of simplicity.  Maybe that will be
	// good enough?
	this->ToStateVector(currentState);
	while (this->currentTime < presentTime)
	{
		VectorN currentStateDerivative(currentState.GetDimension());
		this->ToStateVectorDerivative(currentStateDerivative);

		double timeStep = this->maxTimeStepSize;
		if (this->currentTime + timeStep > presentTime)
			timeStep = presentTime - this->currentTime;

		VectorN nextState(currentState.GetDimension());
		nextState = currentState + currentStateDerivative * timeStep;

		this->FromStateVector(nextState);

		if (!this->InterpenetrationDetected())
		{
			this->currentTime += timeStep;
			currentState = nextState;
		}
		else
		{
			// Do a binary search for the precise time of collision up to the configured tolerance.
			double timeA = this->currentTime;
			double timeB = this->currentTime + timeStep;
			while (timeB - timeA > this->collisionTimeTolerance)
			{
				double midTime = (timeA + timeB) / 2.0;
				timeStep = (timeB - timeA) / 2.0;
				nextState = currentState + currentStateDerivative * timeStep;
				this->FromStateVector(nextState);
				if (this->InterpenetrationDetected())
					timeB = midTime;
				else
				{
					timeA = midTime;
					currentState = nextState;
					this->ToStateVectorDerivative(currentStateDerivative);
				}
			}

			currentState += currentStateDerivative * (timeB - timeA);
			this->FromStateVector(currentState);
			this->currentTime = timeB;

			this->ForAllCollisionPairs([this](PhysicalObject* objectA, PhysicalObject* objectB) -> bool {
				this->ResolveCollision(objectA, objectB);
				return true;
			});
		}
	}
}

bool Simulation::InterpenetrationDetected() const
{
	bool collisionDetected = false;
	const_cast<Simulation*>(this)->ForAllCollisionPairs([&collisionDetected](PhysicalObject* objectA, PhysicalObject* objectB) -> bool {
		collisionDetected = true;
		return false;	// Bail out of iteration at the first detected collision.
	});
	return collisionDetected;
}

void Simulation::ForAllCollisionPairs(CollisionPairFunc collisionPairFunc)
{
	// TODO: Here I'm going to do an O(N^2) algorithm, but this should
	//       eventually be replaced with something closer to O(N log N).

	for (int i = 0; i < (signed)this->physicalObjectArray->size(); i++)
	{
		PhysicalObject* objectA = (*this->physicalObjectArray)[i];
		AxisAlignedBoundingBox boxA;
		if (!objectA->GetBoundingBox(boxA))
			continue;

		for (int j = i + 1; j < (signed)this->physicalObjectArray->size(); j++)
		{
			PhysicalObject* objectB = (*this->physicalObjectArray)[j];
			AxisAlignedBoundingBox boxB;
			if (!objectB->GetBoundingBox(boxB))
				continue;

			AxisAlignedBoundingBox intersection;
			if (!intersection.Intersect(boxA, boxB))
				continue;

			PhysicalObject::CollisionResult result = objectA->IsInCollsionWith(objectB);
			if (result == PhysicalObject::CollisionResult::UNKNOWN)
				result = objectB->IsInCollsionWith(objectA);	// If object A didn't know, deligate to object B.

			if (result != PhysicalObject::CollisionResult::IN_COLLISION)
				continue;

			if (!collisionPairFunc(objectA, objectB))
				return;
		}
	}
}

void Simulation::ToStateVector(VectorN& stateVector) const
{
	int i = 0;
	for (const PhysicalObject* physicalObject : *this->physicalObjectArray)
		physicalObject->GetStateToVector(stateVector, i);
}

void Simulation::FromStateVector(const VectorN& stateVector)
{
	int i = 0;
	for (PhysicalObject* physicalObject : *this->physicalObjectArray)
		physicalObject->SetStateFromVector(stateVector, i);
}

void Simulation::ToStateVectorDerivative(VectorN& stateVectorDerivative)
{
	for (PhysicalObject* physicalObject : *this->physicalObjectArray)
		physicalObject->ZeroNetForcesAndTorques();

	for (ConceptObject* conceptObject : *this->conceptObjectArray)
		conceptObject->ApplyForcesAndTorques(this, this->currentTime);

	int i = 0;
	for (const PhysicalObject* physicalObject : *this->physicalObjectArray)
		physicalObject->CalcStateDerivatives(stateVectorDerivative, i);
}

PhysicsObject* Simulation::FindPhysicsObject(const std::string& name)
{
	PhysicsObjectMap::iterator iter = this->physicsObjectMap->find(name);
	if (iter == this->physicsObjectMap->end())
		return nullptr;

	return iter->second;
}

void Simulation::ResolveCollision(PhysicalObject* objectA, PhysicalObject* objectB)
{
	// Up to this point, we've done the bare minimum amount of work to know if and
	// when two objects are in collision with one another.  Here now we are going to
	// determine how they are in collision and then what to do about it.

	// Deligate this work to the objects themselves.
	if (!objectA->ResolveCollisionWith(objectB))
		objectB->ResolveCollisionWith(objectA);
}