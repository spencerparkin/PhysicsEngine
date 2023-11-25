#pragma once

#include "Vector3.h"

namespace PhysicsEngine
{
	class PHYSICS_ENGINE_API AxisAlignedBoundingBox
	{
	public:
		AxisAlignedBoundingBox();
		AxisAlignedBoundingBox(const Vector3& point);
		AxisAlignedBoundingBox(const Vector3& min, const Vector3& max);
		AxisAlignedBoundingBox(const AxisAlignedBoundingBox& aabb);
		virtual ~AxisAlignedBoundingBox();

		bool IsValid() const;

		double Width() const;
		double Height() const;
		double Depth() const;
		double Volume() const;
		Vector3 Center() const;

		void ScaleAboutCenter(double scale);

		bool ContainsBox(const AxisAlignedBoundingBox& aabb) const;
		bool ContainsPoint(const Vector3& point, double thickness = PHY_ENG_EPS) const;
		bool ContainsInteriorPoint(const Vector3& point, double thickness = PHY_ENG_EPS) const;
		bool ContainsPointOnBoundary(const Vector3& point, double thickness = PHY_ENG_EPS) const;

		void ExpandToIncludePoint(const Vector3& point);

		bool Intersect(const AxisAlignedBoundingBox& aabbA, const AxisAlignedBoundingBox& aabbB);
		bool Merge(const AxisAlignedBoundingBox& aabbA, const AxisAlignedBoundingBox& aabbB);
		bool OverlapsWith(const AxisAlignedBoundingBox& aabb) const;

		void SplitReasonably(AxisAlignedBoundingBox& aabbA, AxisAlignedBoundingBox& aabbB) const;

		bool CalcUVWs(const Vector3& point, Vector3& texCoords) const;
		bool CalcPoint(Vector3& point, const Vector3& texCoords) const;

		typedef std::function<void(const AxisAlignedBoundingBox&)> VisitationFunc;

		// This might be used for integration purposes.
		void VisitSubBoxes(double deltaLength, VisitationFunc visitationFunc);

		Vector3 min, max;
	};
}