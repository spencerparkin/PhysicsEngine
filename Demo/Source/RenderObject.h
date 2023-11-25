#pragma once

#include "Vector3.h"

class RenderObject
{
public:
	RenderObject();
	virtual ~RenderObject();

	virtual void Render() const = 0;

	PhysicsEngine::Vector3 color;
};