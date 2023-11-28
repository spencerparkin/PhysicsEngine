#pragma once

#include <wx/glcanvas.h>
#include "Vector3.h"
#include "Matrix3x3.h"
#include "PolygonMesh.h"
#include "JediForce.h"
#include "JediTorque.h"

class Canvas : public wxGLCanvas
{
public:
	Canvas(wxWindow* parent);
	virtual ~Canvas();

	void HandleKeypresses();

	void OnPaint(wxPaintEvent& event);
	void OnSize(wxSizeEvent& event);
	void OnKeyDown(wxKeyEvent& event);
	void OnKeyUp(wxKeyEvent& event);

private:
	void BindContext();
	void CalcViewFrame(PhysicsEngine::Matrix3x3& viewFrame) const;

	enum class KeyboardMode
	{
		FREE_CAM,
		MAKE_FORCES
	};

	KeyboardMode keyboardMode;
	wxGLContext* context;
	static int attributeList[];
	PhysicsEngine::Vector3 cameraEye;
	PhysicsEngine::Vector3 cameraSubject;
	PhysicsEngine::Vector3 lightPosition;
	PhysicsEngine::JediForce* jediForce;
	PhysicsEngine::JediTorque* jediTorque;
};