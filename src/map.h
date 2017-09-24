#ifndef _MAP_H
#define _MAP_H
#include <string>
#include <vector>

struct PointXY {
    double x, y;
};

struct PointSD {
    double s, d;
};

class Map {
    std::vector<double> map_waypoints_x_;
    std::vector<double> map_waypoints_y_;
    std::vector<double> map_waypoints_s_;
    std::vector<double> map_waypoints_dx_;
    std::vector<double> map_waypoints_dy_;
    double max_s_;

  public:
    Map(const std::string& _file_name, double _max_s);
    virtual ~Map();

    PointSD GetSD(const PointXY& xy, double theta) const;
    PointXY GetXY(const PointSD& sd) const;
    double NormalizeS(double s) const;
    bool InInterval(double s, double from, double to) const;

  private:
    int ClosestWaypoint(const PointXY& xy) const;
    int NextWaypoint(const PointXY& xy, double theta) const;
};

#endif //_MAP_H