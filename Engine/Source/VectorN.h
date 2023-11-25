#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API VectorN
	{
	public:
		VectorN();
		VectorN(const VectorN& vector);
		virtual ~VectorN();

		void SetDimension(int dimension);
		int GetDimension() const;

		void Add(const VectorN& vectorA, const VectorN& vectorB);
		void Subtract(const VectorN& vectorA, const VectorN& vectorB);
		void Multiply(const VectorN& vector, double scalar);

		void operator=(const VectorN& vector);
		void operator+=(const VectorN& vector);
		void operator-=(const VectorN& vector);
		void operator*=(double scalar);
		double operator[](int i) const;
		double& operator[](int i);

	private:
		std::vector<double>* scalarArray;
	};

	PHYSICS_ENGINE_API VectorN operator+(const VectorN& vectorA, const VectorN& vectorB);
	PHYSICS_ENGINE_API VectorN operator-(const VectorN& vectorA, const VectorN& vectorB);
	PHYSICS_ENGINE_API VectorN operator*(const VectorN& vector, double scalar);
	PHYSICS_ENGINE_API VectorN operator*(double scalar, const VectorN& vector);
}