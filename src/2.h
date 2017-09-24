#ifndef _2_h
#define _2_h
#include <vector>
#include <cmath>

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

void calculateNextValues(double car_x, double car_y, double car_yaw,
    const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y,
    std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);

#endif // _2_h