#include "Canvas.h"
#include "Application.h"
#include "Simulation.h"
#include "RenderObject.h"
#include "PhysicsObject.h"
#include "Gravity.h"
#include <gl/GLU.h>

using namespace PhysicsEngine;

int Canvas::attributeList[] = { WX_GL_RGBA, WX_GL_DOUBLEBUFFER, 0 };

Canvas::Canvas(wxWindow* parent) : wxGLCanvas(parent, wxID_ANY, attributeList)
{
	this->cameraEye = Vector3(20.0, 10.0, 20.0);
	this->cameraSubject = Vector3(0.0, 0.0, 0.0);
	this->lightPosition = Vector3(10.0, 10.0, 0.0);

	this->Bind(wxEVT_PAINT, &Canvas::OnPaint, this);
	this->Bind(wxEVT_SIZE, &Canvas::OnSize, this);
	this->Bind(wxEVT_KEY_DOWN, &Canvas::OnKeyDown, this);
	this->Bind(wxEVT_KEY_UP, &Canvas::OnKeyUp, this);

	this->jediForce = wxGetApp().simulation.AddPhysicsObject<JediForce>("jedi_force");
	this->jediForce->SetTarget("boxA");
}

/*virtual*/ Canvas::~Canvas()
{
	delete this->context;
}

void Canvas::OnPaint(wxPaintEvent& event)
{
	this->BindContext();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	double aspectRatio = double(viewport[2]) / double(viewport[3]);
	double fieldOfVision = 50.0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfVision, aspectRatio, 0.1, 10000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(
		this->cameraEye.x,
		this->cameraEye.y,
		this->cameraEye.z,
		this->cameraSubject.x,
		this->cameraSubject.y,
		this->cameraSubject.z,
		0.0,
		1.0,
		0.0);

	glLineWidth(1.5f);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);

	glDisable(GL_LIGHTING);

	// Draw X-axis.
	glColor3d(1.0, 0.0, 0.0);
	glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(1.0, 0.0, 0.0);

	// Draw Y-axis.
	glColor3d(0.0, 1.0, 0.0);
	glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(0.0, 1.0, 0.0);

	// Draw Z-axis.
	glColor3d(0.0, 0.0, 1.0);
	glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(0.0, 0.0, 1.0);

	glEnd();

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	GLfloat lightColor[] = { 1.f, 1.f, 1.f, 1.f };
	GLfloat lightPos[] = { (GLfloat)this->lightPosition.x, (GLfloat)this->lightPosition.y, (GLfloat)this->lightPosition.z, 1.0f };
	GLfloat lightSpec[] = { 0.1f, 0.1f, 0.1f, 0.1f };
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpec);

	Simulation::PhysicsObjectMap& map = wxGetApp().simulation.GetPhysicsObjectMap();
	for(auto pair : map)
	{
		const PhysicsObject* physicsObject = pair.second;
		const RenderObject* renderObject = dynamic_cast<const RenderObject*>(physicsObject);
		if (renderObject)
			renderObject->Render();
	}

	glFlush();

	this->SwapBuffers();
}

void Canvas::OnSize(wxSizeEvent& event)
{
	this->BindContext();

	wxSize size = event.GetSize();
	glViewport(0, 0, size.GetWidth(), size.GetHeight());

	this->Refresh();
}

void Canvas::CalcViewFrame(Matrix3x3& viewFrame) const
{
	Vector3 upVector(0.0, 1.0, 0.0);
	Vector3 zAxis = this->cameraEye - this->cameraSubject;
	zAxis.Normalize();
	Vector3 xAxis = upVector.CrossProduct(zAxis);
	xAxis.Normalize();
	Vector3 yAxis = zAxis.CrossProduct(xAxis);
	viewFrame.SetAxes(xAxis, yAxis, zAxis);
}

void Canvas::HandleKeypresses()
{
	Matrix3x3 viewFrame;
	this->CalcViewFrame(viewFrame);

	Vector3 force(0.0, 0.0, 0.0);

	if (wxGetKeyState(wxKeyCode::WXK_LEFT))
		force += viewFrame * Vector3(-500.0, 0.0, 0.0);

	if (wxGetKeyState(wxKeyCode::WXK_RIGHT))
		force += viewFrame * Vector3(500.0, 0.0, 0.0);

	if (wxGetKeyState(wxKeyCode::WXK_UP))
		force += viewFrame * Vector3(0.0, 500.0, 0.0);

	if (wxGetKeyState(wxKeyCode::WXK_DOWN))
		force += viewFrame * Vector3(0.0, -500.0, 0.0);

	this->jediForce->SetForce(force);
}

void Canvas::OnKeyDown(wxKeyEvent& event)
{
}

void Canvas::OnKeyUp(wxKeyEvent& event)
{
	switch (event.GetKeyCode())
	{
		case 'g':
		case 'G':
		{
			if (wxGetApp().simulation.FindPhysicsObject("gravity"))
				wxGetApp().simulation.RemovePhysicsObject("gravity");
			else
				wxGetApp().simulation.AddPhysicsObject<Gravity>("gravity");
			break;
		}
	}
}

void Canvas::BindContext()
{
	if (!this->context)
		this->context = new wxGLContext(this);

	this->SetCurrent(*this->context);
}