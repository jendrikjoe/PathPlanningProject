#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Helper.h"
#include "Eigen/Core"
#include "Eigen/QR"
#include "json.hpp"
#include "MapTrajectory.h"
#include "TrajectoryGenerator.h"
#include "SpeedController.h"
#include "spline.h"
#include "string.h"
#include "SensorData.h"
#include "BehaviourCommand.h"
#include "BehaviourPlanner.h"
#include "Lanes.h"
#define DEBUG false

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != std::string::npos) {
		return "";
	} else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal std::vectors
	std::vector<double> map_waypoints_x;
	std::vector<double> map_waypoints_y;
	std::vector<double> map_waypoints_s;
	std::vector<double> map_waypoints_dx;
	std::vector<double> map_waypoints_dy;

	// Waypoint map to read from
	std::string map_file_ = "/home/jjordening/git/CarND-Path-Planning-Project/data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	const double max_s = 6945.554;
	const double planningIntervall = 1.2;


	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	std::string line;
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	//Build the controller executing the trajetories with the correct speed
	SpeedController vControl = SpeedController(10,10);
	// Build the object, which is planning the cars behaviour
	BehaviourPlanner planner = BehaviourPlanner();
	// Build the trajectory generator
	TrajectoryGenerator gen = TrajectoryGenerator(planningIntervall, map_waypoints_x, map_waypoints_y,
			map_waypoints_dx, map_waypoints_dy);

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,
				 &map_waypoints_dy, &gen, &vControl, &planningIntervall, &max_s,
				 &planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				//auto sdata = string(data).substr(0, length);
				//std::cout << sdata << std::endl;
				if (length && length > 2 && data[0] == '4' && data[1] == '2') {

					auto s = hasData(data);

					if (s != "") {
						auto j = json::parse(s);

						std::string event = j[0].get<std::string>();

						if (event == "telemetry") {
							// j[1] is the data JSON object

							std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
							// Main car's localization Data
							double car_x = j[1]["x"];
							double car_y = j[1]["y"];
							double car_s = j[1]["s"];
							double car_d = j[1]["d"];
							double car_yaw = j[1]["yaw"];
							car_yaw = Helper::deg2rad(car_yaw);
							car_yaw = car_yaw > M_PI ? car_yaw-2*M_PI: car_yaw;
							double currentSpeed = j[1]["speed"];
							// Factor to convert miles per hour to meters per second
							double speedConversionFactor = 1609.3/3600;
							currentSpeed = currentSpeed*speedConversionFactor;

							// Previous path data given to the Planner
							auto previous_path_x = j[1]["previous_path_x"];
							auto previous_path_y = j[1]["previous_path_y"];
							// Convert the previous path to doubles
							std::vector<double> previousX = std::vector<double>();
							for(int i = 0; i< previous_path_x.size(); i++)
								previousX.push_back(double(previous_path_x[i]));
							std::vector<double> previousY = std::vector<double>();
							for(int i = 0; i< previous_path_y.size(); i++)
								previousY.push_back(double(previous_path_y[i]));
							// Previous path's end s and d values
							double end_path_s = j[1]["end_path_s"];
							double end_path_d = j[1]["end_path_d"];



							// Sensor Fusion Data, a list of all other cars on the same side of the road.
							auto sensor_fusion = j[1]["sensor_fusion"];

							// Convert the sensor data to a SensorData object
							std::vector<SensorData> sensorFusion = std::vector<SensorData>();
							for (int i = 0; i<sensor_fusion.size(); i++) {
								auto data = sensor_fusion[i];
								sensorFusion.push_back(SensorData(data[0], data[1], data[2],
										double(data[3]),
										double(data[4]), data[5], data[6]));
							}
							json msgJson;
							// Declare the MapTrajector, which is later used to publish data to the simulator
							MapTrajectory tra;

							std::vector<double> carCoords {car_x, car_y, car_yaw, car_s, car_d};
							BehaviourCommand command = planner.getNewState(sensorFusion, carCoords, currentSpeed);
							if(previousX.size()!=0) {
								std::vector<double> carCoords {car_x, car_y, car_yaw, car_s, car_d};

								MapTrajectory previousPathLocal = Helper::convertCoordinatesToLocal(previousX,
										previousY, carCoords);


								Trajectory trajectory = gen.getTrajectory(command.getLaneCommand(),
										previousPathLocal, carCoords, currentSpeed);

								tra = vControl.control(trajectory, command.getSpeedCommand(),
										currentSpeed, carCoords, previousPathLocal);
							} else {
								Trajectory trajectory = gen.getInitialTrajectory(command.getLaneCommand(), carCoords);

								 tra = vControl.controlInitial(trajectory, command.getSpeedCommand(),
										currentSpeed, carCoords);
							}
							std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
							std::chrono::duration<double> elapsedSeconds = end-start;
							if(DEBUG) Helper::printVector("xTra: ", tra.getX());
							if(DEBUG) Helper::printVector("yTra: ", tra.getY());
							msgJson["next_x"] = tra.getX();
							msgJson["next_y"] = tra.getY();

							auto msg = "42[\"control\","+ msgJson.dump()+"]";


							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
							//std::this_thread::sleep_for(std::chrono::milliseconds(20));

						}
					} else {
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
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}

