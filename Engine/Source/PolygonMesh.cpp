#include "PolygonMesh.h"
#include "Vector3.h"
#include "Matrix3x3.h"
#include "Plane.h"
#include "LineSegment.h"
#include "AxisAlignedBoundingBox.h"

using namespace PhysicsEngine;

//------------------------------------ PolygonMesh ------------------------------------

PolygonMesh::PolygonMesh()
{
	this->vertexArray = new std::vector<Vector3>();
	this->polygonArray = new std::vector<Polygon*>();
}

/*virtual*/ PolygonMesh::~PolygonMesh()
{
	this->Clear();

	delete this->vertexArray;
	delete this->polygonArray;
}

void PolygonMesh::Clear()
{
	for (Polygon* polygon : *this->polygonArray)
		delete polygon;

	this->polygonArray->clear();
	this->vertexArray->clear();
}

PolygonMesh* PolygonMesh::Clone() const
{
	PolygonMesh* mesh = new PolygonMesh();
	for (const Vector3& vertex : *this->vertexArray)
		mesh->vertexArray->push_back(vertex);
	for (const Polygon* polygon : *this->polygonArray)
		mesh->polygonArray->push_back(polygon->Clone());
	return mesh;
}

bool PolygonMesh::GenerateConvexHull(const std::vector<Vector3>& pointArray)
{
	this->Clear();

	std::list<Triangle> triangleList;

	auto findInitialTetrahedron = [&pointArray, &triangleList]() -> bool
		{
			for (int i = 0; i < (signed)pointArray.size(); i++)
			{
				const Vector3& vertexA = pointArray[i];
				for (int j = i + 1; j < (signed)pointArray.size(); j++)
				{
					const Vector3& vertexB = pointArray[j];
					for (int k = j + 1; k < (signed)pointArray.size(); k++)
					{
						const Vector3& vertexC = pointArray[k];
						for (int l = k + 1; l < (signed)pointArray.size(); l++)
						{
							const Vector3& vertexD = pointArray[l];

							Vector3 xAxis = vertexB - vertexA;
							Vector3 yAxis = vertexC - vertexA;
							Vector3 zAxis = vertexD - vertexA;

							double determinant = xAxis.CrossProduct(yAxis).InnerProduct(zAxis);
							if (determinant > PHY_ENG_SMALL_EPS)
							{
								triangleList.push_back(Triangle{ i, k, j });
								triangleList.push_back(Triangle{ i, j, l });
								triangleList.push_back(Triangle{ j, k, l });
								triangleList.push_back(Triangle{ i, l, k });
								return true;
							}
						}
					}
				}
			}

			return false;
		};

	if (!findInitialTetrahedron())
		return false;

	auto convexHullContainsPoint = [&pointArray, &triangleList](const Vector3& point) -> bool
		{
			for (const Triangle& triangle : triangleList)
			{
				Plane plane;
				triangle.MakePlane(plane, pointArray);
				if (plane.WhichSide(point, PHY_ENG_SMALL_EPS) == Plane::Side::FRONT)
					return false;
			}

			return true;
		};

	auto cancelExistingTriangle = [&triangleList](const Triangle& newTriangle) -> bool
		{
			for (std::list<Triangle>::iterator iter = triangleList.begin(); iter != triangleList.end(); iter++)
			{
				const Triangle& existingTriangle = *iter;
				if (existingTriangle.IsCanceledBy(newTriangle))
				{
					triangleList.erase(iter);
					return true;
				}
			}
			return false;
		};

	std::list<int> pointList;
	for (int i = 0; i < (signed)pointArray.size(); i++)
		pointList.push_back(i);

	while (pointList.size() > 0)
	{
		// Remove all points contained in or on the current convex hull.
		std::list<int>::iterator iter = pointList.begin();
		while (iter != pointList.end())
		{
			std::list<int>::iterator iterNext = iter;
			iterNext++;

			const Vector3& point = pointArray[*iter];
			if (convexHullContainsPoint(point))
				pointList.erase(iter);

			iter = iterNext;
		}

		// We're done once we've processed all the given points.
		if (pointList.size() == 0)
			break;

		// The first point in the list, like all others, should be one we can use to expand the convex hull further.
		int i = *pointList.begin();
		const Vector3& newPoint = pointArray[i];
		std::vector<Triangle> newTriangleArray;
		for (const Triangle& existingTriangle : triangleList)
		{
			Plane plane;
			existingTriangle.MakePlane(plane, pointArray);
			if (plane.WhichSide(newPoint) == Plane::Side::FRONT)
			{
				newTriangleArray.push_back(Triangle{ existingTriangle.vertex[0], existingTriangle.vertex[1], i });
				newTriangleArray.push_back(Triangle{ existingTriangle.vertex[1], existingTriangle.vertex[2], i });
				newTriangleArray.push_back(Triangle{ existingTriangle.vertex[2], existingTriangle.vertex[0], i });
				newTriangleArray.push_back(Triangle{ existingTriangle.vertex[0], existingTriangle.vertex[2], existingTriangle.vertex[1] });
			}
		}

		// Integrate the new triangles into the existing convex hull to build a bigger one.
		for (const Triangle& newTriangle : newTriangleArray)
		{
			// A new triangle is either added to our list, or it cancels an existing triangle.
			if (!cancelExistingTriangle(newTriangle))
				triangleList.push_back(newTriangle);
		}
	}

	// Queue up the surface triangles for processing.
	std::list<Polygon*> polygonQueue;
	for (const Triangle& triangle : triangleList)
	{
		Polygon* polygon = new Polygon();
		for (int i = 0; i < 3; i++)
			polygon->vertexArray->push_back(triangle.vertex[i]);

		polygonQueue.push_back(polygon);
	}

	// Add the polygons to our array, compressing as we go.
	while (polygonQueue.size() > 0)
	{
		std::list<Polygon*>::iterator iter = polygonQueue.begin();
		Polygon* polygon = *iter;
		polygonQueue.erase(iter);
		bool merged = false;

		for (iter = polygonQueue.begin(); iter != polygonQueue.end(); iter++)
		{
			Polygon* otherPolygon = *iter;
			Polygon* mergedPolygon = polygon->MergeWith(otherPolygon, pointArray);
			if (mergedPolygon)
			{
				delete polygon;
				delete otherPolygon;
				polygonQueue.erase(iter);
				polygonQueue.push_back(mergedPolygon);
				merged = true;
				break;
			}
		}

		if (!merged)
			this->polygonArray->push_back(polygon);
	}

	// Populate the vertex list, but only with the vertices we need.
	std::map<int, int> vertexMap;
	for (int i = 0; i < (signed)pointArray.size(); i++)
	{
		for (Polygon* polygon : *this->polygonArray)
		{
			if (polygon->HasVertex(i))
			{
				vertexMap.insert(std::pair<int, int>(i, (int)this->vertexArray->size()));
				this->vertexArray->push_back(pointArray[i]);
				break;
			}
		}
	}

	// Lastly, re-index the polygons.
	for (Polygon* polygon : *this->polygonArray)
		for (int i = 0; i < (signed)polygon->vertexArray->size(); i++)
			(*polygon->vertexArray)[i] = vertexMap.find((*polygon->vertexArray)[i])->second;
	
	return true;
}

bool PolygonMesh::ContainsPoint(const Vector3& point) const
{
	for (Polygon* polygon : *this->polygonArray)
	{
		Plane plane;
		polygon->MakePlane(plane, *this->vertexArray);
		if (plane.WhichSide(point) == Plane::Side::FRONT)
			return false;
	}

	return true;
}

bool PolygonMesh::CalcBoundingBox(AxisAlignedBoundingBox& aabb) const
{
	if (this->vertexArray->size() == 0)
		return false;

	aabb = AxisAlignedBoundingBox((*this->vertexArray)[0]);
	for (int i = 1; i < (signed)this->vertexArray->size(); i++)
		aabb.ExpandToIncludePoint((*this->vertexArray)[i]);

	return true;
}

void PolygonMesh::Translate(const Vector3& translation)
{
	for (Vector3& vertex : *this->vertexArray)
		vertex += translation;

	for (Polygon* polygon : *this->polygonArray)
		polygon->InvalidateCachedPlane();
}

void PolygonMesh::Transform(const Vector3& translation, const Matrix3x3& orientation)
{
	for (Vector3& vertex : *this->vertexArray)
		vertex = orientation * vertex + translation;

	for (Polygon* polygon : *this->polygonArray)
		polygon->InvalidateCachedPlane();
}

int PolygonMesh::GetNumPolygons() const
{
	return (int)this->polygonArray->size();
}

const PolygonMesh::Polygon* PolygonMesh::GetPolygon(int i) const
{
	if (i < 0 || i >= (int)this->polygonArray->size())
		return nullptr;

	return (*this->polygonArray)[i];
}

bool PolygonMesh::CalcCenter(Vector3& center) const
{
	if (this->vertexArray->size() == 0)
		return false;

	center = Vector3(0.0, 0.0, 0.0);
	for (const Vector3& vertex : *this->vertexArray)
		center += vertex;

	center *= 1.0 / double(this->vertexArray->size());
	return true;
}

/*static*/ bool PolygonMesh::ConvexHullsIntersect(const PolygonMesh& meshA, const PolygonMesh& meshB, std::vector<ContactPoint>* contactPointArray /*= nullptr*/)
{
	return meshA.EdgeStrikesFaceOf(meshB, contactPointArray) || meshB.EdgeStrikesFaceOf(meshA, contactPointArray);
}

bool PolygonMesh::EdgeStrikesFaceOf(const PolygonMesh& mesh, std::vector<ContactPoint>* contactPointArray /*= nullptr*/) const
{
	std::vector<Edge> edgeArray;
	this->GenerateEdgeArray(edgeArray);
	for (const Edge& edge : edgeArray)
	{
		LineSegment lineSegment(this->GetVertexArray()[edge.i], this->GetVertexArray()[edge.j]);
		for (const Polygon* polygon : *mesh.polygonArray)
		{
			Vector3 intersectionPoint;
			if (polygon->IntersectedBy(lineSegment, *mesh.vertexArray, intersectionPoint))
			{
				if (!contactPointArray)
					return true;

				Plane plane;
				polygon->MakePlane(plane, *mesh.vertexArray);
				contactPointArray->push_back(ContactPoint{ intersectionPoint, plane.normal });
			}
		}
	}

	if (contactPointArray)
		return contactPointArray->size() > 0;

	return false;
}

void PolygonMesh::GenerateEdgeArray(std::vector<Edge>& edgeArray) const
{
	std::set<Edge> edgeSet;

	for (const Polygon* polygon : *this->polygonArray)
	{
		for (int i = 0; i < (signed)polygon->vertexArray->size(); i++)
		{
			int j = (i + 1) % polygon->vertexArray->size();
			Edge edge{ (*polygon->vertexArray)[i], (*polygon->vertexArray)[j] };
			if (edgeSet.find(edge) == edgeSet.end())
			{
				edgeArray.push_back(edge);
				edgeSet.insert(edge);
			}
		}
	}
}

uint64_t PolygonMesh::Edge::CalcKey() const
{
	if (this->j < this->i)
		return uint64_t(this->j) | (uint64_t(this->i) << 32);

	return uint64_t(this->i) | (uint64_t(this->j) << 32);
}

//------------------------------------ PolygonMesh::Polygon ------------------------------------

PolygonMesh::Polygon::Polygon()
{
	this->vertexArray = new std::vector<int>();
	this->cachedPlane = nullptr;
}

/*virtual*/ PolygonMesh::Polygon::~Polygon()
{
	delete this->vertexArray;
	delete this->cachedPlane;
}

void PolygonMesh::Polygon::Clear()
{
	this->vertexArray->clear();
}

PolygonMesh::Polygon* PolygonMesh::Polygon::Clone() const
{
	Polygon* polygon = new Polygon();
	for (int i : *this->vertexArray)
		polygon->vertexArray->push_back(i);
	return polygon;
}

int PolygonMesh::Polygon::GetNumVertices() const
{
	return (int)this->vertexArray->size();
}

const Vector3& PolygonMesh::Polygon::GetVertex(int i, const PolygonMesh& polygonMesh) const
{
	if (i < 0)
		i = 0;
	else if (i >= (int)this->vertexArray->size())
		i = (int)this->vertexArray->size() - 1;

	return (*polygonMesh.vertexArray)[(*this->vertexArray)[i]];
}

PolygonMesh::Polygon* PolygonMesh::Polygon::MergeWith(const Polygon* polygon, const std::vector<Vector3>& pointArray) const
{
	if (!this->IsCoplanarWith(polygon, pointArray))
		return nullptr;

	struct Edge
	{
		int i, j;
	};

	std::list<Edge> edgeList;
	int cancelationCount = 0;

	auto integrateEdge = [&edgeList, &cancelationCount](Edge newEdge)
		{
			// The new edge might just cancel an existing edge.
			for (std::list<Edge>::iterator iter = edgeList.begin(); iter != edgeList.end(); iter++)
			{
				const Edge& existingEdge = *iter;
				if (existingEdge.i == newEdge.j && existingEdge.j == newEdge.i)
				{
					edgeList.erase(iter);
					cancelationCount++;
					return;
				}
			}

			// At this point we just add the new edge to the list.
			edgeList.push_back(newEdge);
		};

	for (int i = 0; i < (signed)this->vertexArray->size(); i++)
	{
		int j = (i + 1) % this->vertexArray->size();
		integrateEdge(Edge{ (*this->vertexArray)[i], (*this->vertexArray)[j] });
	}

	for (int i = 0; i < (signed)polygon->vertexArray->size(); i++)
	{
		int j = (i + 1) % polygon->vertexArray->size();
		integrateEdge(Edge{ (*polygon->vertexArray)[i], (*polygon->vertexArray)[j] });
	}

	// Reject the merge if no cancelation occurred, which indicates that the two facets don't share at least one edge.
	if (cancelationCount == 0)
		return nullptr;

	// Build the merged polygon.
	Polygon* mergedPolygon = new Polygon();
	if (edgeList.size() > 0)
	{
		mergedPolygon->vertexArray->push_back((*edgeList.begin()).i);
		bool polygonComplete = false;
		while (!polygonComplete)
		{
			int i = (*mergedPolygon->vertexArray)[mergedPolygon->vertexArray->size() - 1];
			for (const Edge& edge : edgeList)
			{
				if (edge.i == i)
				{
					if (edge.j != (*mergedPolygon->vertexArray)[0])
						mergedPolygon->vertexArray->push_back(edge.j);
					else
						polygonComplete = true;
					break;
				}
			}

			// Something has gone wrong in this case.
			if (mergedPolygon->vertexArray->size() > edgeList.size())
			{
				delete mergedPolygon;
				return nullptr;
			}
		}
	}

	return mergedPolygon;
}

bool PolygonMesh::Polygon::AllPointsOnPlane(const Plane& plane, const std::vector<Vector3>& pointArray) const
{
	for (int i = 0; i < (signed)this->vertexArray->size(); i++)
		if (plane.WhichSide(pointArray[(*this->vertexArray)[i]]) != Plane::Side::NEITHER)
			return false;

	return true;
}

bool PolygonMesh::Polygon::IsCoplanarWith(const Polygon* polygon, const std::vector<Vector3>& pointArray) const
{
	Plane plane;
	if (!this->MakePlane(plane, pointArray))
		return false;

	return polygon->AllPointsOnPlane(plane, pointArray);
}

bool PolygonMesh::Polygon::IsCoplanar(const std::vector<Vector3>& pointArray) const
{
	Plane plane;
	if (!this->MakePlane(plane, pointArray))
		return false;

	return this->AllPointsOnPlane(plane, pointArray);
}

void PolygonMesh::Polygon::InvalidateCachedPlane() const
{
	delete this->cachedPlane;
	this->cachedPlane = nullptr;
}

bool PolygonMesh::Polygon::MakePlane(Plane& plane, const std::vector<Vector3>& pointArray) const
{
	if (this->cachedPlane)
	{
		plane = *this->cachedPlane;
		return true;
	}

	this->cachedPlane = new Plane();

	double largestArea = 0.0;
	Triangle bestTriangle{ -1, -1, -1 };

	for (int i = 0; i < (signed)this->vertexArray->size(); i++)
	{
		for (int j = i + 1; j < (signed)this->vertexArray->size(); j++)
		{
			for (int k = j + 1; k < (signed)this->vertexArray->size(); k++)
			{
				Triangle triangle;
				triangle.vertex[0] = (*this->vertexArray)[i];
				triangle.vertex[1] = (*this->vertexArray)[j];
				triangle.vertex[2] = (*this->vertexArray)[k];
				double area = triangle.CalcArea(pointArray);
				if (area > largestArea)
				{
					largestArea = area;
					bestTriangle = triangle;
				}
			}
		}
	}

	if (bestTriangle.vertex[0] == -1)
		return false;

	bestTriangle.MakePlane(plane, pointArray);
	*this->cachedPlane = plane;
	return true;
}

bool PolygonMesh::Polygon::HasVertex(int i) const
{
	for (int vertex : *this->vertexArray)
		if (vertex == i)
			return true;

	return false;
}

bool PolygonMesh::Polygon::IntersectedBy(const LineSegment& lineSegment, const std::vector<Vector3>& pointArray, Vector3& intersectionPoint, double thickness /*= PHY_ENG_SMALL_EPS*/) const
{
	Plane polygonPlane;
	this->MakePlane(polygonPlane, pointArray);
	if (!lineSegment.IntersectWith(polygonPlane, intersectionPoint, thickness))
		return false;

	for (int i = 0; i < (signed)this->vertexArray->size(); i++)
	{
		int j = (i + 1) % this->vertexArray->size();

		const Vector3& edgePointA = pointArray[(*this->vertexArray)[i]];
		const Vector3& edgePointB = pointArray[(*this->vertexArray)[j]];

		Vector3 edgeNormal = (edgePointB - edgePointA).CrossProduct(polygonPlane.normal);
		Plane edgePlane(edgePointA, edgeNormal);

		if (edgePlane.WhichSide(intersectionPoint, thickness) == Plane::Side::FRONT)
			return false;
	}

	return true;
}

//------------------------------------ PolygonMesh::Triangle ------------------------------------

bool PolygonMesh::Triangle::IsCanceledBy(const Triangle& triangle) const
{
	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		if (this->vertex[i] == triangle.vertex[2] &&
			this->vertex[j] == triangle.vertex[1] &&
			this->vertex[k] == triangle.vertex[0])
		{
			return true;
		}
	}

	return false;
}

void PolygonMesh::Triangle::MakePlane(Plane& plane, const std::vector<Vector3>& pointArray) const
{
	const Vector3& pointA = pointArray[this->vertex[0]];
	const Vector3& pointB = pointArray[this->vertex[1]];
	const Vector3& pointC = pointArray[this->vertex[2]];
	Vector3 normal = (pointB - pointA).CrossProduct(pointC - pointA);
	plane = Plane(pointA, normal);
}

double PolygonMesh::Triangle::CalcArea(const std::vector<Vector3>& pointArray) const
{
	const Vector3& pointA = pointArray[this->vertex[0]];
	const Vector3& pointB = pointArray[this->vertex[1]];
	const Vector3& pointC = pointArray[this->vertex[2]];
	return (pointB - pointA).CrossProduct(pointC - pointA).Length() / 2.0;
}

namespace PhysicsEngine
{
	bool operator<(const PolygonMesh::Edge& edgeA, const PolygonMesh::Edge& edgeB)
	{
		uint64_t keyA = edgeA.CalcKey();
		uint64_t keyB = edgeB.CalcKey();

		return keyA < keyB;
	}
}