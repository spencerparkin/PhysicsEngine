#pragma once

#include <wx/frame.h>
#include <wx/timer.h>

class Canvas;

class Frame : public wxFrame
{
public:
	Frame(const wxPoint& pos, const wxSize& size);
	virtual ~Frame();

	void OnExit(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);
	void OnTimer(wxTimerEvent& event);

	enum
	{
		ID_Exit = wxID_HIGHEST,
		ID_About
	};

	Canvas* canvas;
	wxTimer timer;
	bool inTimer;
};