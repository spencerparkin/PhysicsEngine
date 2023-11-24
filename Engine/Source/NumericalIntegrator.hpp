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

		typedef std::function<void(double currentTime, const T& currentValue, T& currentValueDerivative)> DifferentialEquationFunc;

		virtual void Integrate(const T& initialValue, T& finalFinal, double initialTime, double finalTime, DifferentialEquationFunc func) = 0;
	};

	template<typename T>
	class PHYSICS_ENGINE_API EulerIntegrator : public NumericalIntegrator
	{
	public:
		EulerIntegrator(double maxTimeStep = 0.05)
		{
			this->maxTimeStep = maxTimeStep;
		}

		virtual ~EulerIntegrator()
		{
		}

		virtual void Integrate(const T& initialValue, T& finalFinal, double initialTime, double finalTime, DifferentialEquationFunc func) override
		{
			T currentValue = initialValue;
			double currentTime = initialTime;

			while (currentTime < finalTime)
			{
				double deltaTime = this->maxTimeStep;
				if (currentTime + deltaTime > finalTime)
					deltaTime = finalTime - currentTime;

				T currentValueDerivative;
				func(currentTime, currentValue, currentValueDerivative);

				currentValue += currentValueDerivative * deltaTime;
				currentTime += deltaTime;
			}
		}

	private:
		double maxTimeStep;
	};
}