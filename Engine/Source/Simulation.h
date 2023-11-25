#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class PhysicsObject;

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

		// TODO: At some point we need to index all physics objects using
		//       a spacial partitioning datastructure in order to facilitate
		//       efficient collision detection.

		PhysicsObjectMap* physicsObjectMap;

		double currentTime;
		double maxDeltaTime;
	};
}