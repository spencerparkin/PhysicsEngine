#pragma once

#include <wx/glcanvas.h>

class Canvas : public wxGLCanvas
{
public:
	Canvas(wxWindow* parent);
	virtual ~Canvas();

	void OnPaint(wxPaintEvent& event);
	void OnSize(wxSizeEvent& event);
	void OnKeyDown(wxKeyEvent& event);

private:
	void BindContext();

	wxGLContext* context;
	static int attributeList[];
};