#include "Application.h"
#include "Frame.h"
#include "Box.h"
#include <wx/msgdlg.h>

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

	auto box = this->simulation.AddPhysicsObject<Box>();
	if (!box->MakeShape(pointArray, 0.2, [](const Vector3& point) -> double { return 1.0; }))
	{
		wxMessageBox("Failed to create rigid body shape!", "Error", wxOK | wxICON_ERROR, nullptr);
		return false;
	}

	this->frame = new Frame(wxDefaultPosition, wxSize(1200, 800));
	this->frame->Show(true);

	return true;
}

/*virtual*/ int Application::OnExit(void)
{
	return 0;
}