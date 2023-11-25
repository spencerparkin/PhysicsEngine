#pragma once

#include <wx/glcanvas.h>
#include "Vector3.h"
#include "PolygonMesh.h"

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
	PhysicsEngine::Vector3 cameraEye;
	PhysicsEngine::Vector3 cameraSubject;
	PhysicsEngine::Vector3 lightPosition;
};