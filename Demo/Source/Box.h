#pragma once

#include "RigidBody.h"
#include "RenderObject.h"

class Box : public PhysicsEngine::RigidBody, public RenderObject
{
public:
	Box();
	virtual ~Box();

	static Box* Create();

	bool Setup(const PhysicsEngine::Vector3& dimensions, const PhysicsEngine::Vector3& initialPosition);

	virtual void DeleteSelf() override;
	virtual void Render() const override;
};