#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class PhysicsObject;
	class PhysicalObject;
	class ConceptObject;
	class VectorN;

	class PHYSICS_ENGINE_API Simulation
	{
	public:
		Simulation();
		virtual ~Simulation();

		template<typename T>
		T* AddPhysicsObject(const std::string& name)
		{
			PhysicsObject* existingPhysicsObject = this->FindPhysicsObject(name);
			if (existingPhysicsObject)
				return nullptr;

			T* physicsObject = T::Create();
			*physicsObject->name = name;
			this->physicsObjectMap->insert(std::pair<std::string, PhysicsObject*>(name, physicsObject));
			return physicsObject;
		}

		typedef std::map<std::string, PhysicsObject*> PhysicsObjectMap;

		bool RemovePhysicsObject(const std::string& name);
		void Clear();
		void Tick();
		const PhysicsObjectMap& GetPhysicsObjectMap() const { return *this->physicsObjectMap; }
		PhysicsObjectMap& GetPhysicsObjectMap() { return *this->physicsObjectMap; }
		PhysicsObject* FindPhysicsObject(const std::string& name);

	private:

		void ToStateVector(VectorN& stateVector) const;
		void FromStateVector(const VectorN& stateVector);
		void ToStateVectorDerivative(VectorN& stateVectorDerivative);

		bool InterpenetrationDetected() const;
		
		typedef std::function<bool(const PhysicalObject*, const PhysicalObject*)> CollisionPairFunc;
		void ForAllCollisionPairs(CollisionPairFunc collisionPairFunc) const;

		// TODO: At some point we need to index all physics objects using
		//       a spacial partitioning datastructure in order to facilitate
		//       efficient collision detection.

		PhysicsObjectMap* physicsObjectMap;

		double currentTime;				// Each tick, we get our current time caught up with the present time.
		double maxDeltaTime;			// A tick is aborted if the gap between the present time and the current time exceeds this.
		double maxTimeStepSize;			// We catch up to the present time in increments of this size.
		double collisionTimeTolerance;	// Times of collision are determined with an accuracy up to this tolerance.

		std::vector<PhysicalObject*>* physicalObjectArray;
		std::vector<ConceptObject*>* conceptObjectArray;
	};
}