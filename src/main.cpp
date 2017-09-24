#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "util.h"
#include "map.h"
#include "planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main()
{
	uWS::Hub h;

	Map map("../data/highway_map.csv", 6945.554);
	Planner planner(&map, util::milesPerHourToMetersPerSecond(49.5), 50, 0, 1);

	h.onMessage([&planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object

					CarData car_data {
						j[1]["x"],
						j[1]["y"],
						j[1]["s"],
						j[1]["d"],
						util::degToRad(j[1]["yaw"]),
						util::milesPerHourToMetersPerSecond(j[1]["speed"])
					};

					PrevPathData prev_path_data {
						j[1]["previous_path_x"],
						j[1]["previous_path_y"],
						j[1]["end_path_s"],
						j[1]["end_path_d"]
					};

					std::vector<SensorFusionItem> sensor_fusion;
					{
						std::vector<std::vector<double>> sensor_fusion_tmp = j[1]["sensor_fusion"];
						sensor_fusion.reserve(sensor_fusion_tmp.size());
						for(const std::vector<double>& v: sensor_fusion_tmp) {
							sensor_fusion.push_back({(int)v[0], v[1], v[2], v[3], v[4], v[5], v[6]});
						}
					}
					
					planner.SetData(car_data, prev_path_data, sensor_fusion);

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					planner.Plan(next_x_vals, next_y_vals);

					// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
		std::cout << "Disconnected" << std::endl;
		std::exit(0);
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
