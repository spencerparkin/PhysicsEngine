#pragma once

#include "Common.h"

namespace PhysicsEngine
{
	class Vector3;
	class Plane;
	class AxisAlignedBoundingBox;

	class PHYSICS_ENGINE_API PolygonMesh
	{
	public:
		PolygonMesh();
		virtual ~PolygonMesh();

		class PHYSICS_ENGINE_API Polygon
		{
			friend class PolygonMesh;

		public:
			Polygon();
			virtual ~Polygon();

			void Clear();
			Polygon* MergeWith(const Polygon* polygon, const std::vector<Vector3>& pointArray) const;
			bool IsCoplanarWith(const Polygon* polygon, const std::vector<Vector3>& pointArray) const;
			bool IsCoplanar(const std::vector<Vector3>& pointArray) const;
			bool HasVertex(int i) const;
			bool AllPointsOnPlane(const Plane& plane, const std::vector<Vector3>& pointArray) const;
			bool MakePlane(Plane& plane, const std::vector<Vector3>& pointArray) const;

		private:
			std::vector<int>* vertexArray;
		};

		void Clear();
		bool GenerateConvexHull(const std::vector<Vector3>& pointArray);
		bool ContainsPoint(const Vector3& point) const;		// This assumes the mesh forms a convex hull.
		bool CalcBoundingBox(AxisAlignedBoundingBox& aabb) const;
		void Translate(const Vector3& translationDelta);

	private:

		struct Triangle
		{
		public:
			bool IsCanceledBy(const Triangle& triangle) const;
			void MakePlane(Plane& plane, const std::vector<Vector3>& pointArray) const;
			double CalcArea(const std::vector<Vector3>& pointArray) const;

			int vertex[3];
		};

		std::vector<Vector3>* vertexArray;
		std::vector<Polygon*>* polygonArray;
	};
}