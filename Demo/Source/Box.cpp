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

bool Box::Setup(const Vector3& dimensions, const Vector3& initialPosition)
{
	std::vector<Vector3> pointArray;
	pointArray.push_back(Vector3(-3.0, -3.0, -3.0) + Vector3(2.0, 2.0, 0.0));
	pointArray.push_back(Vector3(3.0, -3.0, -3.0) + Vector3(2.0, 2.0, 0.0));
	pointArray.push_back(Vector3(3.0, 3.0, -3.0) + Vector3(2.0, 2.0, 0.0));
	pointArray.push_back(Vector3(-3.0, 3.0, -3.0) + Vector3(2.0, 2.0, 0.0));
	pointArray.push_back(Vector3(-3.0, -3.0, 3.0) + Vector3(2.0, 2.0, 0.0));
	pointArray.push_back(Vector3(3.0, -3.0, 3.0) + Vector3(2.0, 2.0, 0.0));
	pointArray.push_back(Vector3(3.0, 3.0, 3.0) + Vector3(2.0, 2.0, 0.0));
	pointArray.push_back(Vector3(-3.0, 3.0, 3.0) + Vector3(2.0, 2.0, 0.0));

	if (!this->MakeShape(pointArray, 0.2, [](const Vector3& point) -> double { return 1.0; }))
		return false;

	this->SetLocation(initialPosition);
	return true;
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

	GLfloat diffuseColor[] = { (GLfloat)this->color.x, (GLfloat)this->color.y, (GLfloat)this->color.z, 1.0f };
	GLfloat specularColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat ambientColor[] = { (GLfloat)this->color.x, (GLfloat)this->color.y, (GLfloat)this->color.z, 1.0f };
	GLfloat shininess[] = { 30.0f };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseColor);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specularColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientColor);
	glMaterialfv(GL_FRONT, GL_SHININESS, shininess);

	glMaterialfv(GL_BACK, GL_DIFFUSE, diffuseColor);
	glMaterialfv(GL_BACK, GL_SPECULAR, specularColor);
	glMaterialfv(GL_BACK, GL_AMBIENT, ambientColor);
	glMaterialfv(GL_BACK, GL_SHININESS, shininess);

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
			const Vector3& localVertex = polygon->GetVertex(j, mesh);
			Vector3 worldVertex = orientation * localVertex + position;

			glNormal3dv(&plane.normal.x);
			glVertex3dv(&worldVertex.x);
		}

		glEnd();
	}
}