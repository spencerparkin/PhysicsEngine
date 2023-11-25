#include "NumericalIntegrator.h"
#include "VectorN.h"

using namespace PhysicsEngine;

//--------------------------------- NumericalIntegrator ---------------------------------

NumericalIntegrator::NumericalIntegrator()
{
}

/*virtual*/ NumericalIntegrator::~NumericalIntegrator()
{
}

//--------------------------------- EulerIntegrator ---------------------------------

EulerIntegrator::EulerIntegrator(double timeStep)
{
	this->timeStep = timeStep;
}

/*virtual*/ EulerIntegrator::~EulerIntegrator()
{
}

/*virtual*/ void EulerIntegrator::Integrate(const VectorN& initialState, VectorN& finalState, double initialTime, double finalTime, DifferentialEquationFunc differentialEquationFunc)
{
	VectorN currentState = initialState;
	double currentTime = initialTime;

	while (currentTime < finalTime)
	{
		double deltaTime = this->timeStep;
		if (currentTime + deltaTime > finalTime)
			deltaTime = finalTime - currentTime;

		VectorN currentStateDerivative;
		differentialEquationFunc(currentState, currentTime, currentStateDerivative);

		currentState += currentStateDerivative * currentTime;
	}
}		