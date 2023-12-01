#pragma once

#include "PhysicalObject.h"
#include "Plane.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API GroundPlane : public PhysicalObject
	{
	public:
		GroundPlane();
		virtual ~GroundPlane();

		static GroundPlane* Create();

		const Plane& GetPlane() const { return this->plane; }
		void SetPlane(const Plane& plane) { this->plane = plane; }

		virtual Vector3 GetCenter() const override;
		virtual void SetCenter(const Vector3& center) override;
		virtual CollisionResult IsInCollsionWith(const PhysicalObject* physicalObject) const override;
		virtual CollisionResolution ResolveCollisionWith(PhysicalObject* physicalObject) override;
		virtual bool GetBoundingBox(AxisAlignedBoundingBox& boundingBox) const override;

	private:
		Plane plane;
	};
}