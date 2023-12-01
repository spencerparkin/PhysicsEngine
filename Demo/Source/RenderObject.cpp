#include <wx/glcanvas.h>
#include "RenderObject.h"

using namespace PhysicsEngine;

RenderObject::RenderObject()
{
	this->color = Vector3(0.5, 0.5, 0.5);
}

/*virtual*/ RenderObject::~RenderObject()
{
}

void RenderObject::IssueColor() const
{
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
}