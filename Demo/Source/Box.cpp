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
	pointArray.push_back(Vector3(-dimensions.x / 2.0, -dimensions.y / 2.0, -dimensions.z / 2.0));
	pointArray.push_back(Vector3(dimensions.x / 2.0, -dimensions.y / 2.0, -dimensions.z / 2.0));
	pointArray.push_back(Vector3(dimensions.x / 2.0, dimensions.y / 2.0, -dimensions.z / 2.0));
	pointArray.push_back(Vector3(-dimensions.x / 2.0, dimensions.y / 2.0, -dimensions.z / 2.0));
	pointArray.push_back(Vector3(-dimensions.x / 2.0, -dimensions.y / 2.0, dimensions.z / 2.0));
	pointArray.push_back(Vector3(dimensions.x / 2.0, -dimensions.y / 2.0, dimensions.z / 2.0));
	pointArray.push_back(Vector3(dimensions.x / 2.0, dimensions.y / 2.0, dimensions.z / 2.0));
	pointArray.push_back(Vector3(-dimensions.x / 2.0, dimensions.y / 2.0, dimensions.z / 2.0));

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

	glEnable(GL_LIGHTING);
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
		const Vector3& localNormal = plane.normal;

		glBegin(GL_POLYGON);
		
		int numVertices = polygon->GetNumVertices();
		for (int j = 0; j < numVertices; j++)
		{
			const Vector3& localVertex = polygon->GetVertex(j, mesh);
			Vector3 worldNormal = orientation * localNormal;
			Vector3 worldVertex = orientation * localVertex + position;

			glNormal3dv(&worldNormal.x);
			glVertex3dv(&worldVertex.x);
		}

		glEnd();
	}

	glDisable(GL_LIGHTING);
	glLineWidth(1.5f);
	glBegin(GL_LINES);
	glColor3f(1.0f, 1.0f, 1.0f);

	Vector3 meshCenter;
	mesh.CalcCenter(meshCenter);

	std::vector<PolygonMesh::Edge> edgeArray;
	mesh.GenerateEdgeArray(edgeArray);
	for (const PolygonMesh::Edge& edge : edgeArray)
	{
		Vector3 localVertexA = mesh.GetVertexArray()[edge.i];
		Vector3 localVertexB = mesh.GetVertexArray()[edge.j];

		localVertexA = meshCenter + (localVertexA - meshCenter) * 1.001;
		localVertexB = meshCenter + (localVertexB - meshCenter) * 1.001;

		Vector3 worldVertexA = orientation * localVertexA + position;
		Vector3 worldVertexB = orientation * localVertexB + position;

		glVertex3dv(&worldVertexA.x);
		glVertex3dv(&worldVertexB.x);
	}

	glEnd();
}