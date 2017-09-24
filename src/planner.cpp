#include "planner.h"

#include <algorithm>
#include "spline.h"
#include "util.h"
#include "map.h"


Planner::Planner(
    const Map* _map,
    double _max_allowed_speed,    
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
, max_allowed_speed_(_max_allowed_speed)
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
, current_target_speed_(0)
, current_target_lane_(1)
{
    sensor_fusion_.reserve(_sensor_fusion.size());
    for(const std::vector<double>& item : _sensor_fusion) {
        sensor_fusion_.push_back({(int)item[0], item[1], item[2], item[3], item[4], item[5], item[6]});
    }
}

Planner::~Planner(){
    // Empty
}

void Planner::Plan(std::vector<double>& path_x, std::vector<double>& path_y)
{
    path_x = {};
    path_y = {};

    bool danger = IsThereACarInFrontOfUs(current_target_lane_);
    if(danger) {
        if(current_target_lane_ >= 1 && !IsThereACarInFrontOfUs(current_target_lane_ - 1, true))
        {
            current_target_lane_ -= 1;
        }
        else if(current_target_lane_ <= 1 && !IsThereACarInFrontOfUs(current_target_lane_ + 1, true))
        {
            current_target_lane_ += 1;
        } else 
        {
            current_target_speed_ -= 0.1; //m/s
        }
    } else if(current_target_speed_ + 0.1 <= max_allowed_speed_) {
        current_target_speed_ += 0.1;
    }

    current_target_speed_ = std::max(current_target_speed_, 0.1); // 

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

    PointSD origin_sd = map_->GetSD({origin_x, origin_y}, origin_yaw);
    PointXY ref_p0 = map_->GetXY({origin_sd.s + 30, 2.0 + 4.0 * current_target_lane_}); // FIXME magic number
    PointXY ref_p1 = map_->GetXY({origin_sd.s + 60, 2.0 + 4.0 * current_target_lane_}); // FIXME magic number
    PointXY ref_p2 = map_->GetXY({origin_sd.s + 90, 2.0 + 4.0 * current_target_lane_}); // FIXME magic number
    
    ref_points_x.push_back(ref_p0.x);
    ref_points_x.push_back(ref_p1.x);
    ref_points_x.push_back(ref_p2.x);

    ref_points_y.push_back(ref_p0.y);
    ref_points_y.push_back(ref_p1.y);
    ref_points_y.push_back(ref_p2.y);

    std::vector<double> spline_x;
    std::vector<double> spline_y;
    int n_points_to_generate = std::max(50-(int)path_x.size(), 1); // FIXME magic number
    GenerateSpline(ref_points_x, ref_points_y, n_points_to_generate, origin_x, origin_y, origin_yaw, spline_x, spline_y);

    std::copy(spline_x.begin(), spline_x.end(), std::back_inserter(path_x));
    std::copy(spline_y.begin(), spline_y.end(), std::back_inserter(path_y));
}

bool Planner::IsThereACarInFrontOfUs(int lane, bool strict) const
{
    double origin_s = (int)prev_path_x_.size() > 0 ? prev_path_end_s_ : car_s_;

    for(const SensorFusionItem& item: sensor_fusion_)
    {
        if(item.d >= 4.0 * lane && item.d <= 4.0 * (lane+1))
        {
            double other_car_speed = std::sqrt(item.vx * item.vx + item.vy * item.vy);
            double other_car_s = item.s; 
            // project to future
            other_car_s += (double)prev_path_x_.size() * other_car_speed / points_per_sec_;
            double safety_zone = strict ? 10 : 0;
            if(other_car_s >= origin_s - safety_zone && other_car_s <= origin_s + 30)
            { // TODO ezt majd modulo kell számolni
                return true;
            }
        }
    }

    return false;
}

void Planner::GenerateSpline(
    std::vector<double> ref_points_x, 
    std::vector<double> ref_points_y,
    int n_points_to_generate,
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
    double N =  points_per_sec_ * target_dist / current_target_speed_;

    for(int i = 0; i<=n_points_to_generate; i++) //
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
