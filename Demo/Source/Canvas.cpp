#include "Canvas.h"
#include "Application.h"
#include "Simulation.h"
#include "RenderObject.h"
#include "PhysicsObject.h"
#include "RigidBody.h"
#include "Gravity.h"
#include <gl/GLU.h>

using namespace PhysicsEngine;

int Canvas::attributeList[] = { WX_GL_RGBA, WX_GL_DOUBLEBUFFER, 0 };

Canvas::Canvas(wxWindow* parent) : wxGLCanvas(parent, wxID_ANY, attributeList)
{
	this->keyboardMode = KeyboardMode::FREE_CAM;
	this->cameraEye = Vector3(0.0, 0.0, 10.0);
	this->cameraSubject = Vector3(0.0, 0.0, 0.0);
	this->lightPosition = Vector3(10.0, 20.0, 50.0);

	this->Bind(wxEVT_PAINT, &Canvas::OnPaint, this);
	this->Bind(wxEVT_SIZE, &Canvas::OnSize, this);
	this->Bind(wxEVT_KEY_DOWN, &Canvas::OnKeyDown, this);
	this->Bind(wxEVT_KEY_UP, &Canvas::OnKeyUp, this);

	this->jediForce = wxGetApp().simulation.AddPhysicsObject<JediForce>("jedi_force");
	this->jediForce->SetTarget("boxA");

	this->jediTorque = wxGetApp().simulation.AddPhysicsObject<JediTorque>("jedi_torque");
	this->jediTorque->SetTarget("boxA");
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

	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);

	for (const PolygonMesh::ContactPoint& contactPoint : this->contactPointArray)
	{
		Vector3 tip = contactPoint.point + contactPoint.normal;

		glVertex2dv(&contactPoint.point.x);
		glVertex2dv(&tip.x);
	}

	glEnd();

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

	switch (keyboardMode)
	{
		case KeyboardMode::MAKE_FORCES:
		{
			Vector3 force(0.0, 0.0, 0.0);
			Vector3 torque(0.0, 0.0, 0.0);
			double strength = 50.0;

			// Apply a force.
			if (wxGetKeyState(wxKeyCode::WXK_LEFT))
				force += viewFrame * Vector3(-strength, 0.0, 0.0);		// -X
			if (wxGetKeyState(wxKeyCode::WXK_RIGHT))
				force += viewFrame * Vector3(strength, 0.0, 0.0);		// +X
			if (wxGetKeyState(wxKeyCode::WXK_DOWN))
				force += viewFrame * Vector3(0.0, -strength, 0.0);		// -Y
			if (wxGetKeyState(wxKeyCode::WXK_UP))
				force += viewFrame * Vector3(0.0, strength, 0.0);		// +Y	

			// Apply a torque.
			if (wxGetKeyState(wxKeyCode::WXK_NUMPAD2))
				torque += viewFrame * Vector3(-strength, 0.0, 0.0);		// -X
			if (wxGetKeyState(wxKeyCode::WXK_NUMPAD8))
				torque += viewFrame * Vector3(strength, 0.0, 0.0);		// +X
			if (wxGetKeyState(wxKeyCode::WXK_NUMPAD6))
				torque += viewFrame * Vector3(0.0, -strength, 0.0);		// -Y
			if (wxGetKeyState(wxKeyCode::WXK_NUMPAD4))
				torque += viewFrame * Vector3(0.0, strength, 0.0);		// +Y

			this->jediForce->SetForce(force);
			this->jediTorque->SetTorque(torque);
			break;
		}
		case KeyboardMode::FREE_CAM:
		{
			double sensativity = 0.2;

			if (wxGetKeyState(wxKeyCode::WXK_ALT))
			{
				Vector3 subjectDelta(0.0, 0.0, 0.0);

				if (wxGetKeyState(wxKeyCode::WXK_LEFT))
					subjectDelta += viewFrame * Vector3(-sensativity, 0.0, 0.0);
				if (wxGetKeyState(wxKeyCode::WXK_RIGHT))
					subjectDelta += viewFrame * Vector3(sensativity, 0.0, 0.0);
				if (wxGetKeyState(wxKeyCode::WXK_DOWN))
					subjectDelta += viewFrame * Vector3(0.0, -sensativity, 0.0);
				if (wxGetKeyState(wxKeyCode::WXK_UP))
					subjectDelta += viewFrame * Vector3(0.0, sensativity, 0.0);

				Vector3 lookVector = this->cameraSubject - this->cameraEye;
				double length;
				lookVector.Normalize(&length);
				this->cameraSubject += subjectDelta;
				lookVector = this->cameraSubject - this->cameraEye;
				lookVector.Normalize();
				lookVector *= length;
			}
			else
			{
				Vector3 cameraDelta(0.0, 0.0, 0.0);

				if (wxGetKeyState(wxKeyCode::WXK_LEFT))
					cameraDelta += viewFrame * Vector3(-sensativity, 0.0, 0.0);
				if (wxGetKeyState(wxKeyCode::WXK_RIGHT))
					cameraDelta += viewFrame * Vector3(sensativity, 0.0, 0.0);
				if (wxGetKeyState(wxKeyCode::WXK_DOWN))
					cameraDelta += viewFrame * Vector3(0.0, 0.0, sensativity);
				if (wxGetKeyState(wxKeyCode::WXK_UP))
					cameraDelta += viewFrame * Vector3(0.0, 0.0, -sensativity);

				this->cameraEye += cameraDelta;
				this->cameraSubject += cameraDelta;
			}

			break;
		}
	}
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
		case 'a':
		case 'A':
		{
			this->jediForce->SetTarget("boxA");
			this->jediTorque->SetTarget("boxA");
			break;
		}
		case 'b':
		case 'B':
		{
			this->jediForce->SetTarget("boxB");
			this->jediTorque->SetTarget("boxB");
			break;
		}
		case 'm':
		case 'M':
		{
			if (this->keyboardMode == KeyboardMode::FREE_CAM)
				this->keyboardMode = KeyboardMode::MAKE_FORCES;
			else
				this->keyboardMode = KeyboardMode::FREE_CAM;
			break;
		}
		case 'c':
		case 'C':
		{
			RigidBody* boxA = dynamic_cast<RigidBody*>(wxGetApp().simulation.FindPhysicsObject("boxA"));
			RigidBody* boxB = dynamic_cast<RigidBody*>(wxGetApp().simulation.FindPhysicsObject("boxB"));
			if (boxA && boxB)
			{
				PolygonMesh* meshA = boxA->GetMesh().Clone();
				PolygonMesh* meshB = boxB->GetMesh().Clone();

				meshA->Transform(boxA->GetLocation(), boxA->GetOrientation());
				meshB->Transform(boxB->GetLocation(), boxB->GetOrientation());

				this->contactPointArray.clear();
				PolygonMesh::CalculateContactPoints(*meshA, *meshB, this->contactPointArray);

				if (contactPointArray.size() == 0)
				{
				}

				delete meshA;
				delete meshB;
			}
		}
	}
}

void Canvas::BindContext()
{
	if (!this->context)
		this->context = new wxGLContext(this);

	this->SetCurrent(*this->context);
}