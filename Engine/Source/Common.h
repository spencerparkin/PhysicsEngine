#pragma once

#if defined PHYSICS_ENGINE_EXPORT
#	define PHYSICS_ENGINE_API			__declspec(dllexport)
#elif defined PHYSICS_ENGINE_IMPORT
#	define PHYSICS_ENGINE_API			__declspec(dllimport)
#else
#	define PHYSICS_ENGINE_API
#endif

#include <string>
#include <list>
#include <map>
#include <vector>
#include <functional>
#include <math.h>
#include <time.h>

#define PHY_ENG_EPS				1e-6
#define PHY_ENG_PI				3.1415926536
#define PHY_ENG_SQUARED(x)		((x) * (x))
#define PHY_ENG_SIGN(x)			((x) < 0 ? -1.0 : 1.0)
#define PHY_ENG_MIN(x, y)		((x) < (y) ? (x) : (y))
#define PHY_ENG_MAX(x, y)		((x) > (y) ? (x) : (y))