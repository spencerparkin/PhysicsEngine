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
		T* AddPhysicsObject()
		{
			T* physicsObject = T::Create();
			this->physicsObjectArray->push_back(physicsObject);
			return physicsObject;
		}

		void Clear();
		void Tick(double deltaTime);

	private:

		// TODO: At some point we may need to put everything into a
		//       spacial partitioning data-structure, but this is fine
		//       for now.
		std::vector<PhysicsObject*>* physicsObjectArray;
	};
}