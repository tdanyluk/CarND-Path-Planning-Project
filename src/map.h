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

  public:
    Map(const std::string& file_name_);
    virtual ~Map();

    PointSD GetSD(const PointXY& xy, double theta) const;
    PointXY GetXY(const PointSD& sd) const;

  private:
    int ClosestWaypoint(const PointXY& xy) const;
    int NextWaypoint(const PointXY& xy, double theta) const;

};

#endif //_MAP_H