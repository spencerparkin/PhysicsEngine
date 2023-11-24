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
		typedef std::function<void(T& currentValue)> StabilizeFunc;

		virtual void Integrate(const T& initialValue, T& finalFinal, double initialTime, double finalTime, DifferentialEquationFunc differentialEquationFunc) = 0;

		StabilizeFunc stabilizeFunc;
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

		virtual void Integrate(const T& initialValue, T& finalFinal, double initialTime, double finalTime, DifferentialEquationFunc differentialEquationFunc) override
		{
			T currentValue = initialValue;
			double currentTime = initialTime;

			while (currentTime < finalTime)
			{
				double deltaTime = this->maxTimeStep;
				if (currentTime + deltaTime > finalTime)
					deltaTime = finalTime - currentTime;

				T currentValueDerivative;
				differentialEquationFunc(currentTime, currentValue, currentValueDerivative);

				currentValue += currentValueDerivative * deltaTime;
				currentTime += deltaTime;

				// If we're dealing with quaternions, this should re-normalize the quaternion.
				// If we're dealing with rotation matrices, then this should re-orthonormalize the matrix.
				if (this->stabilizeFunc)
					this->stabilizeFunc(currentValue);
			}
		}

	private:
		double maxTimeStep;
	};
}