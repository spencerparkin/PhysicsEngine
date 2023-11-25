#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class VectorN;

	class PHYSICS_ENGINE_API NumericalIntegrator
	{
	public:
		NumericalIntegrator();
		virtual ~NumericalIntegrator();

		typedef std::function<void(const VectorN& currentState, double currentTime, VectorN& currentStateDerivative)> DifferentialEquationFunc;

		virtual void Integrate(const VectorN& initialState, VectorN& finalState, double initialTime, double finalTime, DifferentialEquationFunc differentialEquationFunc) = 0;
	};

	class PHYSICS_ENGINE_API EulerIntegrator : public NumericalIntegrator
	{
	public:
		EulerIntegrator(double timeStep);
		virtual ~EulerIntegrator();

		virtual void Integrate(const VectorN& initialState, VectorN& finalState, double initialTime, double finalTime, DifferentialEquationFunc differentialEquationFunc) override;

		double timeStep;
	};
}