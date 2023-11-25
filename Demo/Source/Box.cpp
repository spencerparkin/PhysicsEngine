#include "Box.h"

Box::Box()
{
}

/*virtual*/ Box::~Box()
{
}

/*static*/ Box* Box::Create()
{
	return new Box();
}

/*virtual*/ void Box::DeleteSelf()
{
	delete this;
}

/*virtual*/ void Box::Render() const
{
}