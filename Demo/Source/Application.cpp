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

	auto boxA = this->simulation.AddPhysicsObject<Box>("boxA");
	if (!boxA->Setup(Vector3(1.0, 1.5, 2.0), Vector3(-3.0, 0.0, 0.0)))
	{
		wxMessageBox("Failed to setup box A!", "Error", wxOK | wxICON_ERROR, nullptr);
	}

	auto boxB = this->simulation.AddPhysicsObject<Box>("boxB");
	if (!boxB->Setup(Vector3(0.5, 1.5, 1.8), Vector3(3.0, 0.0, 0.0)))
	{
		wxMessageBox("Failed to setup box B!", "Error", wxOK | wxICON_ERROR, nullptr);
	}

	this->frame = new Frame(wxDefaultPosition, wxSize(1200, 800));
	this->frame->Show(true);

	return true;
}

/*virtual*/ int Application::OnExit(void)
{
	return 0;
}