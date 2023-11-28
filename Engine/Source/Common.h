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
#include <set>
#include <vector>
#include <functional>
#include <math.h>
#include <time.h>
#include <memory>

#define PHY_ENG_SMALL_EPS		1e-6
#define PHY_ENG_FAT_EPS			1e-4
#define PHY_ENG_OBESE_EPS		1e-2
#define PHY_ENG_PI				3.1415926536
#define PHY_ENG_SQUARED(x)		((x) * (x))
#define PHY_ENG_SIGN(x)			((x) < 0 ? -1.0 : 1.0)
#define PHY_ENG_MIN(x, y)		((x) < (y) ? (x) : (y))
#define PHY_ENG_MAX(x, y)		((x) > (y) ? (x) : (y))