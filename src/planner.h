#ifndef _PLANNER_H
#define _PLANNER_H
#include <vector>

class Map;

struct CarData {
    CarData()
    : x(0), y(0), s(0), d(0), yaw(0), speed(0)
    {}

    CarData(double _x, double _y, double _s, double _d, double _yaw, double _speed)
    : x(_x), y(_y), s(_s), d(_d), yaw(_yaw), speed(_speed)
    {}
    
    double x, y, s, d, yaw, speed; // yaw: rad, speed: m/s
};

struct PrevPathData {
    PrevPathData()
    : x(), y(), end_s(0), end_d(0)
    {}
      
    PrevPathData(const std::vector<double>& _x, const std::vector<double>& _y, double _end_s, double _end_d)
    : x(_x), y(_y), end_s(_end_s), end_d(_end_d)
    {}
        
    int size() const { return x.size(); }

    std::vector<double> x;
    std::vector<double> y;
    double end_s;
    double end_d;
};

struct SensorFusionItem {
    int id;
    double x, y, vx, vy, s, d;
};

class Planner
{
    const Map *map_;
    double speed_limit_;
    int points_per_sec_;

    double current_target_speed_;
    int current_target_lane_;

    CarData car_;
    PrevPathData prev_path_;
    std::vector<SensorFusionItem> sensor_fusion_;
public:
    Planner(
        const Map* _map,
        double _speed_limit,
        int _points_per_sec,
        double _current_target_speed,
        int _current_target_lane
    );

    void SetData(const CarData& car, const PrevPathData& prev_path, const std::vector<SensorFusionItem>& sensor_fusion);

    void Plan(std::vector<double>& path_x, std::vector<double>& path_y);

    virtual ~Planner();

private:
    bool IsLaneClear(int lane) const;

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