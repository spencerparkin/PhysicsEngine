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

	auto box = this->simulation.AddPhysicsObject<Box>("boxA");
	if (!box->Setup(Vector3(3.0, 3.0, 3.0), Vector3(0.0, 0.0, 0.0)))
	{
		wxMessageBox("Failed to setup box!", "Error", wxOK | wxICON_ERROR, nullptr);
	}

	this->frame = new Frame(wxDefaultPosition, wxSize(1200, 800));
	this->frame->Show(true);

	return true;
}

/*virtual*/ int Application::OnExit(void)
{
	return 0;
}