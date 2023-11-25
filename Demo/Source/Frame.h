#pragma once

#include <wx/frame.h>

class Frame : public wxFrame
{
public:
	Frame(const wxPoint& pos, const wxSize& size);
	virtual ~Frame();

	void OnExit(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);

	enum
	{
		ID_Exit = wxID_HIGHEST,
		ID_About
	};
};