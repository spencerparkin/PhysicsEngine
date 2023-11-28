#include "VectorN.h"

using namespace PhysicsEngine;

VectorN::VectorN()
{
	this->scalarArray = new std::vector<double>();
}

VectorN::VectorN(const VectorN& vector)
{
	this->scalarArray = new std::vector<double>();
	*this = vector;
}

VectorN::VectorN(int dimension)
{
	this->scalarArray = new std::vector<double>();
	this->SetDimension(dimension);
}

/*virtual*/ VectorN::~VectorN()
{
	delete this->scalarArray;
}

void VectorN::SetDimension(int dimension)
{
	this->scalarArray->resize(dimension);
}

int VectorN::GetDimension() const
{
	return (int)this->scalarArray->size();
}

void VectorN::Add(const VectorN& vectorA, const VectorN& vectorB)
{
	int N = PHY_ENG_MIN(vectorA.GetDimension(), vectorB.GetDimension());
	this->SetDimension(N);
	for (int i = 0; i < N; i++)
		(*this->scalarArray)[i] = vectorA[i] + vectorB[i];
}

void VectorN::Subtract(const VectorN& vectorA, const VectorN& vectorB)
{
	int N = PHY_ENG_MIN(vectorA.GetDimension(), vectorB.GetDimension());
	this->SetDimension(N);
	for (int i = 0; i < N; i++)
		(*this->scalarArray)[i] = vectorA[i] - vectorB[i];
}

void VectorN::Multiply(const VectorN& vector, double scalar)
{
	int N = vector.GetDimension();
	this->SetDimension(N);
	for (int i = 0; i < N; i++)
		(*this->scalarArray)[i] = vector[i] * scalar;
}

void VectorN::operator=(const VectorN& vector)
{
	this->SetDimension(vector.GetDimension());
	for (int i = 0; i < this->GetDimension(); i++)
		(*this->scalarArray)[i] = (*vector.scalarArray)[i];
}

void VectorN::operator+=(const VectorN& vector)
{
	this->Add(*this, vector);
}

void VectorN::operator-=(const VectorN& vector)
{
	this->Subtract(*this, vector);
}

void VectorN::operator*=(double scalar)
{
	this->Multiply(*this, scalar);
}

double VectorN::operator[](int i) const
{
	return (*this->scalarArray)[i];
}

double& VectorN::operator[](int i)
{
	return (*this->scalarArray)[i];
}

namespace PhysicsEngine
{
	VectorN operator+(const VectorN& vectorA, const VectorN& vectorB)
	{
		VectorN result;
		result.Add(vectorA, vectorB);
		return result;
	}

	VectorN operator-(const VectorN& vectorA, const VectorN& vectorB)
	{
		VectorN result;
		result.Subtract(vectorA, vectorB);
		return result;
	}

	VectorN operator*(const VectorN& vector, double scalar)
	{
		VectorN result;
		result.Multiply(vector, scalar);
		return result;
	}

	VectorN operator*(double scalar, const VectorN& vector)
	{
		VectorN result;
		result.Multiply(vector, scalar);
		return result;
	}
}