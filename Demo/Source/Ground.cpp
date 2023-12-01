#include <wx/glcanvas.h>
#include "Ground.h"
#include "Vector3.h"

using namespace PhysicsEngine;

Ground::Ground()
{
	this->SetPlane(Plane(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0)));
}

/*virtual*/ Ground::~Ground()
{
}

/*static*/ Ground* Ground::Create()
{
	return new Ground();
}

/*virtual*/ void Ground::DeleteSelf()
{
	delete this;
}

/*virtual*/ void Ground::Render() const
{
	Vector3 center = this->GetPlane().CalcCenter();
	Vector3 zAxis = this->GetPlane().normal;
	Vector3 yAxis;
	yAxis.SetOrthogonalTo(zAxis);
	yAxis.Normalize();
	Vector3 xAxis = yAxis.CrossProduct(zAxis);

	glEnable(GL_LIGHTING);
	
	this->IssueColor();

	double extent = 100.0;

	Vector3 vertex[4];
	vertex[0] = center - xAxis * extent - yAxis * extent;
	vertex[1] = center + xAxis * extent - yAxis * extent;
	vertex[2] = center + xAxis * extent + yAxis * extent;
	vertex[3] = center - xAxis * extent + yAxis * extent;

	glBegin(GL_QUADS);

	for (int i = 0; i < 4; i++)
	{
		glNormal3dv(&zAxis.x);
		glVertex3dv(&vertex[i].x);
	}

	glEnd();
}