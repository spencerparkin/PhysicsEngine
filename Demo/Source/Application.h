#pragma once

#include <wx/setup.h>
#include <wx/app.h>
#include "Simulation.h"

class Frame;

class Application : public wxApp
{
public:
	Application();
	virtual ~Application();

	virtual bool OnInit(void) override;
	virtual int OnExit(void) override;

	Frame* frame;

	PhysicsEngine::Simulation simulation;
};

wxDECLARE_APP(Application);