#pragma once

class RenderObject
{
public:
	RenderObject();
	virtual ~RenderObject();

	virtual void Render() const = 0;
};