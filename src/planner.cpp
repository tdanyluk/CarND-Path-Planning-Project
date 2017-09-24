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
    
    std::vector<double> pts_x;
    std::vector<double> pts_y;

    double ref_x = car_x_;
    double ref_y = car_y_;
    double ref_yaw = util::degToRad(car_yaw_);

    int prev_size = (int)prev_path_x_.size();

    if(prev_size < 2) {
        double prev_car_x = ref_x - std::cos(ref_yaw);
        double prev_car_y = ref_y - std::sin(ref_yaw);

        pts_x.push_back(prev_car_x);
        pts_x.push_back(ref_x);
        
        pts_y.push_back(prev_car_y);
        pts_y.push_back(ref_y);
    } else {
        ref_x = prev_path_x_[prev_size-1];
        ref_y = prev_path_y_[prev_size-1];

        double ref_x_prev = prev_path_x_[prev_size-2];
        double ref_y_prev = prev_path_y_[prev_size-2];

        pts_x.push_back(ref_x_prev);
        pts_x.push_back(ref_x);
        
        pts_y.push_back(ref_y_prev);
        pts_y.push_back(ref_y);

        ref_yaw = std::atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    }

    PointSD frenet = map_->GetSD({ref_x, ref_y}, ref_yaw);
    auto ref_s = frenet.s;
    auto ref_d = frenet.d;

    int lane = 1;
    PointXY next_wp0 = map_->GetXY({ref_s + 30, 2.0+4.0*lane});
    PointXY next_wp1 = map_->GetXY({ref_s + 60, 2.0+4.0*lane});
    PointXY next_wp2 = map_->GetXY({ref_s + 90, 2.0+4.0*lane});
    
    pts_x.push_back(next_wp0.x);
    pts_x.push_back(next_wp1.x);
    pts_x.push_back(next_wp2.x);

    pts_y.push_back(next_wp0.y);
    pts_y.push_back(next_wp1.y);
    pts_y.push_back(next_wp2.y);

    UnshiftUnrotate(pts_x, pts_y, ref_x, ref_y, ref_yaw);
    
    tk::spline s;

    s.set_points(pts_x, pts_y);

    for(int i = 0; i<(int)prev_path_x_.size(); i++)
    {
        path_x.push_back(prev_path_x_[i]);
        path_y.push_back(prev_path_y_[i]);
    }

    double target_x = 30;
    double target_y = s(target_y);
    double target_dist = util::distance(0, 0, target_x, target_y);
    double N =  points_per_sec_ * target_dist / target_speed_;

    std::vector<double> spline_x;
    std::vector<double> spline_y;
    for(int i = 0; i<=50-prev_size; i++) //
    {
        double x = (i + 1) * target_x / N;
        double y = s(x);
        spline_x.push_back(x);
        spline_y.push_back(y);
    }

    ShiftRotate(spline_x, spline_y, ref_x, ref_y, ref_yaw);
    std::copy(spline_x.begin(), spline_x.end(), std::back_inserter(path_x));
    std::copy(spline_y.begin(), spline_y.end(), std::back_inserter(path_y));
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

