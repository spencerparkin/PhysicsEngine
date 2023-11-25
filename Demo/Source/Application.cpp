#include "Application.h"
#include "Frame.h"
#include "RigidBody.h"

wxIMPLEMENT_APP(Application);

using namespace PhysicsEngine;

Application::Application()
{
	this->frame = nullptr;
}

/*virtual*/ Application::~Application()
{
}

/*virtual*/ bool Application::OnInit(void)
{
	if (!wxApp::OnInit())
		return false;

	std::vector<Vector3> pointArray;
	pointArray.push_back(Vector3(-3.0, -3.0, -3.0));
	pointArray.push_back(Vector3(3.0, -3.0, -3.0));
	pointArray.push_back(Vector3(3.0, 3.0, -3.0));
	pointArray.push_back(Vector3(-3.0, 3.0, -3.0));
	pointArray.push_back(Vector3(-3.0, -3.0, 3.0));
	pointArray.push_back(Vector3(3.0, -3.0, 3.0));
	pointArray.push_back(Vector3(3.0, 3.0, 3.0));
	pointArray.push_back(Vector3(-3.0, 3.0, 3.0));

	auto rigidBody = this->simulation.AddPhysicsObject<RigidBody>();
	rigidBody->MakeShape(pointArray, 0.2, [](const Vector3& point) -> double { return 1.0; });

	this->frame = new Frame(wxDefaultPosition, wxSize(1200, 800));
	this->frame->Show(true);

	return true;
}

/*virtual*/ int Application::OnExit(void)
{
	return 0;
}