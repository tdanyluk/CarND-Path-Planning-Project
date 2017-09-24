#include "planner.h"

#include <algorithm>
#include "spline.h"
#include "util.h"
#include "map.h"


Planner::Planner(
    const Map* _map,
    double _target_speed,    
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
    const std::vector<std::vector<double>>& _sensor_fusion)
: map_(_map)
, target_speed_(_target_speed)
, points_per_sec_(_points_per_sec)
, car_x_(_car_x)
, car_y_(_car_y)
, car_s_(_car_s)
, car_d_(_car_d)
, car_yaw_(_car_yaw)
, car_speed_(_car_speed)
, prev_path_x_(_prev_path_x)
, prev_path_y_(_prev_path_y)
, prev_path_end_s_(_prev_path_end_s)
, prev_path_end_d_(_prev_path_end_d)
, sensor_fusion_()
{
    sensor_fusion_.reserve(_sensor_fusion.size());
    for(const std::vector<double>& item : _sensor_fusion) {
        sensor_fusion_.push_back({(int)item[0], item[1], item[2], item[3], item[4], item[5], item[6]});
    }
}

Planner::~Planner(){
    // Empty
}

void Planner::Plan(std::vector<double>& path_x, std::vector<double>& path_y) const
{
    path_x = {};
    path_y = {};

    std::copy(prev_path_x_.begin(), prev_path_x_.end(), std::back_inserter(path_x));
    std::copy(prev_path_y_.begin(), prev_path_y_.end(), std::back_inserter(path_y));
    
    double prev_x = 0;
    double prev_y = 0;
    double origin_x = 0;
    double origin_y = 0;
    double origin_yaw = 0;

    double prev_path_size = (int)prev_path_x_.size();
    if(prev_path_size < 2) {
        prev_x = car_x_ - std::cos(car_yaw_);
        prev_y = car_y_ - std::sin(car_yaw_);
        origin_x = car_x_;
        origin_y = car_y_;
        origin_yaw = car_yaw_;
    } else {
        prev_x = prev_path_x_[prev_path_size-2];
        prev_y = prev_path_y_[prev_path_size-2];
        origin_x = prev_path_x_[prev_path_size-1];
        origin_y = prev_path_y_[prev_path_size-1];
        origin_yaw = std::atan2(origin_y-prev_y, origin_x-prev_x);
    }

    std::vector<double> ref_points_x;
    std::vector<double> ref_points_y;

    ref_points_x.push_back(prev_x);
    ref_points_x.push_back(origin_x);

    ref_points_y.push_back(prev_y);
    ref_points_y.push_back(origin_y);

    const int lane = 1;    
    PointSD origin_sd = map_->GetSD({origin_x, origin_y}, origin_yaw);
    PointXY ref_p0 = map_->GetXY({origin_sd.s + 30, 2.0 + 4.0 * lane}); // FIXME magic number
    PointXY ref_p1 = map_->GetXY({origin_sd.s + 60, 2.0 + 4.0 * lane}); // FIXME magic number
    PointXY ref_p2 = map_->GetXY({origin_sd.s + 90, 2.0 + 4.0 * lane}); // FIXME magic number
    
    ref_points_x.push_back(ref_p0.x);
    ref_points_x.push_back(ref_p1.x);
    ref_points_x.push_back(ref_p2.x);

    ref_points_y.push_back(ref_p0.y);
    ref_points_y.push_back(ref_p1.y);
    ref_points_y.push_back(ref_p2.y);

    std::vector<double> spline_x;
    std::vector<double> spline_y;
    int nPointsToGenerate = std::max(50-(int)path_x.size(), 1); // FIXME magic number
    GenerateSpline(ref_points_x, ref_points_y, nPointsToGenerate, origin_x, origin_y, origin_yaw, spline_x, spline_y);

    std::copy(spline_x.begin(), spline_x.end(), std::back_inserter(path_x));
    std::copy(spline_y.begin(), spline_y.end(), std::back_inserter(path_y));
}

void Planner::GenerateSpline(
    std::vector<double> ref_points_x, 
    std::vector<double> ref_points_y,
    int nPointsToGenerate,
    double x_origin,
    double y_origin,
    double yaw,
    std::vector<double>& spline_x, 
    std::vector<double>& spline_y) const 
{
    spline_x = {};
    spline_y = {};

    UnshiftUnrotate(ref_points_x, ref_points_y, x_origin, y_origin, yaw);

    tk::spline s;
    s.set_points(ref_points_x, ref_points_y);

    double target_x = 30;
    double target_y = s(target_y);
    double target_dist = util::distance(0, 0, target_x, target_y);
    double N =  points_per_sec_ * target_dist / target_speed_;

    for(int i = 0; i<=nPointsToGenerate; i++) //
    {
        double x = (i + 1) * target_x / N;
        double y = s(x);
        spline_x.push_back(x);
        spline_y.push_back(y);
    }

    ShiftRotate(spline_x, spline_y, x_origin, y_origin, yaw);
}

void Planner::UnshiftUnrotate(std::vector<double>& xs, std::vector<double>& ys, double x_origin, double y_origin, double yaw) const
{
    assert(xs.size() == ys.size());

    for(int i = 0; i<(int)xs.size(); i++){
        double shift_x = xs[i] - x_origin;
        double shift_y = ys[i] - y_origin;

        xs[i] = shift_x * std::cos(-yaw) - shift_y * std::sin(-yaw);
        ys[i] = shift_x * std::sin(-yaw) + shift_y * std::cos(-yaw);
    }
}

void Planner::ShiftRotate(std::vector<double>& xs, std::vector<double>& ys, double x_origin, double y_origin, double yaw) const
{
    assert(xs.size() == ys.size());
    
    for(int i = 0; i<(int)xs.size(); i++){
        double x = xs[i];
        double y = ys[i];

        xs[i] = x_origin + x * std::cos(yaw) - y * std::sin(yaw);
        ys[i] = y_origin + x * std::sin(yaw) + y * std::cos(yaw);
    }
}
