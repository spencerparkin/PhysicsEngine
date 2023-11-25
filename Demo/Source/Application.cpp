#include "Application.h"
#include "Frame.h"

wxIMPLEMENT_APP(Application);

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

	this->frame = new Frame(wxDefaultPosition, wxSize(1200, 800));
	this->frame->Show(true);

	return true;
}

/*virtual*/ int Application::OnExit(void)
{
	return 0;
}