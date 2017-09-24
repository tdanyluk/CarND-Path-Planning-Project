#include "2.h"

using namespace std;

void calculateNextValues(double car_x, double car_y, double car_yaw,
    const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y,
    std::vector<double>& next_x_vals, std::vector<double>& next_y_vals)
{
    next_x_vals = {};
    next_y_vals = {};

    double pos_x = 0;
    double pos_y = 0;
    double angle = 0;
    int path_size = previous_path_x.size();

    for(int i = 0; i < path_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if(path_size == 0)
    {
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);
    }
    else
    {
        pos_x = previous_path_x[path_size-1];
        pos_y = previous_path_y[path_size-1];

        double pos_x2 = previous_path_x[path_size-2];
        double pos_y2 = previous_path_y[path_size-2];
        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    }

    double dist_inc = 0.5;
    for(int i = 0; i < 50-path_size; i++)
    {    
        next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
        next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
        pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
        pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    }
}
