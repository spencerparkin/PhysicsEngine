#include "Frame.h"
#include <wx/menu.h>
#include <wx/aboutdlg.h>

Frame::Frame(const wxPoint& pos, const wxSize& size) : wxFrame(nullptr, wxID_ANY, "Physics Engine Demo", pos, size)
{
	wxMenu* demoMenu = new wxMenu();
	demoMenu->Append(new wxMenuItem(demoMenu, ID_Exit, "Exit", "Get the hell outta here."));

	wxMenu* helpMenu = new wxMenu();
	helpMenu->Append(new wxMenuItem(helpMenu, ID_About, "About", "Show the about dialog."));

	wxMenuBar* menuBar = new wxMenuBar();
	menuBar->Append(demoMenu, "Demo");
	menuBar->Append(helpMenu, "Help");
	this->SetMenuBar(menuBar);

	this->Bind(wxEVT_MENU, &Frame::OnExit, this, ID_Exit);
	this->Bind(wxEVT_MENU, &Frame::OnAbout, this, ID_About);

	this->SetStatusBar(new wxStatusBar(this));
}

/*virtual*/ Frame::~Frame()
{
}

void Frame::OnExit(wxCommandEvent& event)
{
	this->Close(true);
}

void Frame::OnAbout(wxCommandEvent& event)
{
	wxAboutDialogInfo aboutDialogInfo;

	aboutDialogInfo.SetName("Physics Engine Demo");
	aboutDialogInfo.SetVersion("1.0");
	aboutDialogInfo.SetDescription("This is mainly testing ground for the physics engine library.");
	aboutDialogInfo.SetCopyright("Copyright (C) 2023 -- Spencer T. Parkin <SpencerTParkin@gmail.com>");

	wxAboutBox(aboutDialogInfo);
}