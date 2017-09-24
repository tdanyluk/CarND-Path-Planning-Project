#ifndef _UTIL_H
#define _UTIL_H

#include <cmath>

namespace util
{

inline double pi() { return 3.14159265358979323846; }

inline double degToRad(double x) { return x * pi() / 180; }

inline double radToDeg(double x) { return x * 180 / pi(); }

inline double distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

} // namespace util

#endif // _UTIL_H