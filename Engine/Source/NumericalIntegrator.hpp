#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	template<typename T>
	class PHYSICS_ENGINE_API NumericalIntegrator
	{
	public:
		NumericalIntegrator()
		{
		}

		virtual ~NumericalIntegrator()
		{
		}

		typedef std::function<void(const T& currentState, double currentTime, T& currentStateDerivative)> DifferentialEquationFunc;

		virtual void Integrate(const T& initialState, T& finalState, double initialTime, double finalTime, DifferentialEquationFunc differentialEquationFunc) = 0;
	};

	template<typename T>
	class PHYSICS_ENGINE_API EulerIntegrator : public NumericalIntegrator<T>
	{
	public:
		EulerIntegrator(double timeStep)
		{
			this->timeStep = timeStep;
		}

		virtual ~EulerIntegrator()
		{
		}

		typedef std::function<void(const T& currentState, double currentTime, T& currentStateDerivative)> DifferentialEquationFunc;

		virtual void Integrate(const T& initialState, T& finalState, double initialTime, double finalTime, DifferentialEquationFunc differentialEquationFunc) override
		{
			T currentState = initialState;
			double currentTime = initialTime;

			while (currentTime < finalTime)
			{
				double deltaTime = this->timeStep;
				if (currentTime + deltaTime > finalTime)
					deltaTime = finalTime - currentTime;

				T currentStateDerivative;
				differentialEquationFunc(currentState, currentTime, currentStateDerivative);

				currentState += currentStateDerivative * deltaTime;
				currentTime += deltaTime;
			}
		}

	private:
		double timeStep;
	};
}