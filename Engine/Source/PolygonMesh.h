#pragma once

#include "Common.h"
#include "Vector3.h"

namespace PhysicsEngine
{
	class Matrix3x3;
	class Plane;
	class AxisAlignedBoundingBox;
	class LineSegment;

	class PHYSICS_ENGINE_API PolygonMesh
	{
	public:
		PolygonMesh();
		virtual ~PolygonMesh();

		struct ContactPoint
		{
			Vector3 point;
			Vector3 normal;
		};

		class PHYSICS_ENGINE_API Polygon
		{
			friend class PolygonMesh;

		public:
			Polygon();
			virtual ~Polygon();

			void Clear();
			Polygon* Clone() const;
			Polygon* MergeWith(const Polygon* polygon, const std::vector<Vector3>& pointArray) const;
			bool IsCoplanarWith(const Polygon* polygon, const std::vector<Vector3>& pointArray) const;
			bool IsCoplanar(const std::vector<Vector3>& pointArray) const;
			bool HasVertex(int i) const;
			bool AllPointsOnPlane(const Plane& plane, const std::vector<Vector3>& pointArray) const;
			bool MakePlane(Plane& plane, const std::vector<Vector3>& pointArray) const;
			int GetNumVertices() const;
			const Vector3& GetVertex(int i, const PolygonMesh& polygonMesh) const;
			void InvalidateCachedPlane() const;
			bool IntersectedBy(const LineSegment& lineSegment, const std::vector<Vector3>& pointArray, Vector3& intersectionPoint, double thickness = PHY_ENG_SMALL_EPS) const;

		private:
			mutable Plane* cachedPlane;
			std::vector<int>* vertexArray;
		};

		struct Edge
		{
			int i, j;

			uint64_t CalcKey() const;
		};

		void Clear();
		PolygonMesh* Clone() const;
		bool GenerateConvexHull(const std::vector<Vector3>& pointArray);
		bool ContainsPoint(const Vector3& point) const;		// This assumes the mesh forms a convex hull.
		bool CalcBoundingBox(AxisAlignedBoundingBox& aabb) const;
		void Translate(const Vector3& translation);
		void Transform(const Vector3& translation, const Matrix3x3& orientation);
		int GetNumPolygons() const;
		const Polygon* GetPolygon(int i) const;
		const std::vector<Vector3>& GetVertexArray() const { return *this->vertexArray; }
		bool CalcCenter(Vector3& center) const;
		void GenerateEdgeArray(std::vector<Edge>& edgeArray) const;
		static bool ConvexHullsIntersect(const PolygonMesh& meshA, const PolygonMesh& meshB, std::vector<ContactPoint>* contactPointArray = nullptr);		// This assumes both meshes are convex hulls.
		bool EdgeStrikesFaceOf(const PolygonMesh& mesh, std::vector<ContactPoint>* contactPointArray = nullptr) const;

	private:

		static void CompressContactPointArray(const std::vector<ContactPoint>& contactPointArray, std::vector<ContactPoint>& compressedContactPointArray);

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

	bool operator<(const PolygonMesh::Edge& edgeA, const PolygonMesh::Edge& edgeB);
}