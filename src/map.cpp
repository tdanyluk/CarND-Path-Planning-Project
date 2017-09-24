#include "map.h"

#include <fstream>
#include <sstream>
#include "util.h"

Map::Map(const std::string &file_name_)
{
    std::ifstream f(file_name_);

    std::string line;
    while (std::getline(f, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x_.push_back(x);
        map_waypoints_y_.push_back(y);
        map_waypoints_s_.push_back(s);
        map_waypoints_dx_.push_back(d_x);
        map_waypoints_dy_.push_back(d_y);
    }
}

Map::~Map()
{
    // Empty
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
PointSD Map::GetSD(const PointXY& xy, double theta) const
{
    const auto& maps_x = map_waypoints_x_;
    const auto& maps_y = map_waypoints_y_;
    double x = xy.x;
    double y = xy.y;

	int next_wp = NextWaypoint(xy, theta);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = (int)maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = util::distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = util::distance(center_x, center_y, x_x, x_y);
	double centerToRef = util::distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += util::distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += util::distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
PointXY Map::GetXY(const PointSD& sd) const
{
    const auto& maps_x = map_waypoints_x_;
    const auto& maps_y = map_waypoints_y_;
    const auto& maps_s = map_waypoints_s_;
    double s = sd.s;
    double d = sd.d;

	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < ((int)maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % (int)maps_x.size();

	double heading = std::atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * std::cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * std::sin(heading);

	double perp_heading = heading - util::pi() / 2;

	double x = seg_x + d * std::cos(perp_heading);
	double y = seg_y + d * std::sin(perp_heading);

	return {x, y};
}

int Map::ClosestWaypoint(const PointXY& xy) const
{
    const auto& maps_x = map_waypoints_x_;
    const auto& maps_y = map_waypoints_y_;
    double x = xy.x;
    double y = xy.y;

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < (int)maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = util::distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int Map::NextWaypoint(const PointXY& xy, double theta) const
{
    const auto& maps_x = map_waypoints_x_;
    const auto& maps_y = map_waypoints_y_;
    double x = xy.x;
    double y = xy.y;

	int closestWaypoint = ClosestWaypoint(xy);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = std::atan2((map_y - y), (map_x - x));

	double angle = std::abs(theta - heading);

	if (angle > util::pi() / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;
}