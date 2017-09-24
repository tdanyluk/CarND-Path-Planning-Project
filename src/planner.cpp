#include "planner.h"

#include <cmath>
#include "spline.h"
#include "util.h"
#include "map.h"


Planner::Planner(
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
    const std::vector<std::vector<double>>& _sensor_fusion)
: map_(_map)
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

    for(int i = 0; i<pts_x.size(); i++){
        double shift_x = pts_x[i] - ref_x;
        double shift_y = pts_y[i] - ref_y;

        pts_x[i] = shift_x*std::cos(-ref_yaw)-shift_y*std::sin(-ref_yaw);
        pts_y[i] = shift_x*std::sin(-ref_yaw)+shift_y*std::cos(-ref_yaw);
    }

    tk::spline s;

    s.set_points(pts_x, pts_y);

    for(int i = 0; i<(int)prev_path_x_.size(); i++)
    {
        path_x.push_back(prev_path_x_[i]);
        path_y.push_back(prev_path_y_[i]);
    }

    double target_x = 30;
    double target_y = s(target_y);
    double target_dist = util::distance(0,0,target_x, target_y);
    double ref_vel = 49.5;
    double N = target_dist/(0.02*ref_vel/2.24); // 2.24: miles/h to meters/s, every 0.02 sec the car moves

    double x_add_on = 0;
    for(int i = 0; i<=50-prev_size; i++)//
    {
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * std::cos(ref_yaw) - y_ref*std::sin(ref_yaw);
        y_point = x_ref * std::sin(ref_yaw) + y_ref*std::cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        path_x.push_back(x_point);
        path_y.push_back(y_point);
    }
}


Planner::~Planner(){
    // Empty
}

