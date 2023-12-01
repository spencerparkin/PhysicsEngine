#pragma once

#include "RenderObject.h"
#include "GroundPlane.h"

class Ground : public PhysicsEngine::GroundPlane, public RenderObject
{
public:
	Ground();
	virtual ~Ground();

	static Ground* Create();

	virtual void DeleteSelf() override;
	virtual void Render() const override;
};