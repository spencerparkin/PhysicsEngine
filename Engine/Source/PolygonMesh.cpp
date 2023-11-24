#include "PolygonMesh.h"
#include "Vector3.h"
#include "Plane.h"
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
							if (determinant > PHY_ENG_EPS)
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
				if (plane.WhichSide(point, PHY_ENG_EPS) == Plane::Side::FRONT)
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

void PolygonMesh::Translate(const Vector3& translationDelta)
{
	for (Vector3& vertex : *this->vertexArray)
		vertex += translationDelta;
}

//------------------------------------ PolygonMesh::Polygon ------------------------------------

PolygonMesh::Polygon::Polygon()
{
	this->vertexArray = new std::vector<int>();
}

/*virtual*/ PolygonMesh::Polygon::~Polygon()
{
	delete this->vertexArray;
}

void PolygonMesh::Polygon::Clear()
{
	this->vertexArray->clear();
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

bool PolygonMesh::Polygon::MakePlane(Plane& plane, const std::vector<Vector3>& pointArray) const
{
	double largestArea = 0.0;
	Triangle bestTriangle{ -1, -1, -1 };

	for (int i = 0; i < (signed)this->vertexArray->size(); i++)
	{
		for (int j = i + 1; j < (signed)this->vertexArray->size(); j++)
		{
			for (int k = j + 1; k < (signed)this->vertexArray->size(); k++)
			{
				Triangle triangle{ i, j, k };
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
	return true;
}

bool PolygonMesh::Polygon::HasVertex(int i) const
{
	for (int vertex : *this->vertexArray)
		if (vertex == i)
			return true;

	return false;
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