#include <wx/glcanvas.h>
#include "Box.h"
#include "PolygonMesh.h"
#include "Plane.h"

using namespace PhysicsEngine;

Box::Box()
{
}

/*virtual*/ Box::~Box()
{
}

/*static*/ Box* Box::Create()
{
	return new Box();
}

/*virtual*/ void Box::DeleteSelf()
{
	delete this;
}

/*virtual*/ void Box::Render() const
{
	const PolygonMesh& mesh = this->GetMesh();
	const Vector3& position = this->GetLocation();
	const Matrix3x3& orientation = this->GetOrientation();

	glColor3dv(&this->color.x);

	int numPolygons = mesh.GetNumPolygons();
	for (int i = 0; i < numPolygons; i++)
	{
		const PolygonMesh::Polygon* polygon = mesh.GetPolygon(i);

		Plane plane;
		polygon->MakePlane(plane, mesh.GetVertexArray());

		glBegin(GL_POLYGON);
		
		int numVertices = polygon->GetNumVertices();
		for (int j = 0; j < numVertices; j++)
		{
			const Vector3& vertex = polygon->GetVertex(j, mesh);

			glNormal3dv(&plane.normal.x);
			glVertex3dv(&vertex.x);
		}

		glEnd();
	}
}