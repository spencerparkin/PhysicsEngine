#include "Frame.h"
#include "Canvas.h"
#include "Application.h"
#include <wx/menu.h>
#include <wx/aboutdlg.h>
#include <wx/sizer.h>

Frame::Frame(const wxPoint& pos, const wxSize& size) : wxFrame(nullptr, wxID_ANY, "Physics Engine Demo", pos, size), timer(this)
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
	this->Bind(wxEVT_TIMER, &Frame::OnTimer, this);

	this->SetStatusBar(new wxStatusBar(this));

	this->canvas = new Canvas(this);

	wxBoxSizer* boxSizer = new wxBoxSizer(wxVERTICAL);
	boxSizer->Add(this->canvas, 1, wxALL | wxGROW, 0);
	this->SetSizer(boxSizer);

	this->inTimer = false;
	this->timer.Start(0);
}

/*virtual*/ Frame::~Frame()
{
}

void Frame::OnTimer(wxTimerEvent& event)
{
	if (this->inTimer)
		return;

	this->inTimer = true;

	wxGetApp().simulation.Tick();

	this->inTimer = false;
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
	aboutDialogInfo.SetDescription("This is mainly a testing ground for the physics engine library.");
	aboutDialogInfo.SetCopyright("Copyright (C) 2023 -- Spencer T. Parkin <SpencerTParkin@gmail.com>");

	wxAboutBox(aboutDialogInfo);
}