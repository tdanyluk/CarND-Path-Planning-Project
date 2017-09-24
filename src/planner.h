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
    double max_allowed_speed_;
    int points_per_sec_;

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
    double current_target_speed_; //TODO make private
    int current_target_lane_;

    Planner(
        const Map* _map,
        double _target_speed, // m/s
        int _points_per_sec,
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

    void Plan(std::vector<double>& path_x, std::vector<double>& path_y);

    virtual ~Planner();

private:
    bool IsThereACarInFrontOfUs(int lane, bool strict = false) const;

    void GenerateSpline(
        std::vector<double> ref_points_x, 
        std::vector<double> ref_points_y,
        int n_points_to_generate,
        double x_origin,
        double y_origin,
        double yaw,
        std::vector<double>& spline_x, 
        std::vector<double>& spline_y
    ) const;

    void UnshiftUnrotate(std::vector<double>& xs, std::vector<double>& ys, double x_origin, double y_origin, double yaw) const;
    void ShiftRotate(std::vector<double>& xs, std::vector<double>& ys, double x_origin, double y_origin, double yaw) const;
};

#endif //_PLANNER_H