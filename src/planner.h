#ifndef _PLANNER_H
#define _PLANNER_H
#include <vector>

class Map;

struct SensorFusionItem {
    int id;
    double x, y, vx, vy, s, d;
};

class Planner
{
    const Map *map_;

    double car_x_; // meter
    double car_y_;
    double car_s_;
    double car_d_;
    double car_yaw_; // radian
    double car_speed_; //? 
    std::vector<double> prev_path_x_;
    std::vector<double> prev_path_y_;
    double prev_path_end_s_;
    double prev_path_end_d_;
    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    std::vector<SensorFusionItem> sensor_fusion_;

  public:
    Planner(
        const Map* _map,
        double _car_x,
        double _car_y,
        double _car_s,
        double _car_d,
        double _car_yaw,
        double _car_speed,
        const std::vector<double>& _prev_path_x,
        const std::vector<double>& _prev_path_y,
        double _prev_path_end_s,
        double _prev_path_end_d,
        const std::vector<std::vector<double>>& _sensor_fusion
    );

    void Plan(std::vector<double>& path_x, std::vector<double>& path_y) const;

    virtual ~Planner();
};

#endif //_PLANNER_H