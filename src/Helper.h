/*
 * Helper.h
 *
 *  Created on: Jul 19, 2017
 *      Author: jjordening
 */

#ifndef SRC_HELPER_H_
#define SRC_HELPER_H_


#include <vector>
#include <math.h>
#include "MapTrajectory.h"
#include "spline.h"
#include "Trajectory.h"
#include "SensorData.h"

/**
 * This class just contains static functions which come in handy,
 * when converting representations.
 */
class Helper {
public:
	static std::vector<double> getFrenet(double x, double y, double theta,
			std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_s);
	static int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);
	static MapTrajectory convertFrenetToMap(Trajectory &tra, const std::vector<double> &maps_s,
			const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &carCoords);
	static MapTrajectory convertFrenetToLocalFrame(Trajectory &tra, const std::vector<double> &maps_s,
				const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &carCoords);
	static tk::spline getSplineCurve(std::vector<double> &x, std::vector<double> &y);
	static tk::spline getSplineCurve(std::vector<std::vector<double>> &xy);
	static std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
			const std::vector<double> &maps_x, const std::vector<double> &maps_y);
	static double distance(double x1, double y1, double x2, double y2);
	static int closestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);
	static std::vector<double> convertCoordinatesToLocal(double x, double y,
			std::vector<double> carCoords);
	static MapTrajectory convertCoordinatesToLocal(std::vector<double> x,
			std::vector<double> y, std::vector<double> carCoords);
	static std::vector<SensorData> convertCoordinatesToLocal(std::vector<SensorData> sensorFusion,
			std::vector<double> carCoords);
	static std::vector<double> convertCoordinatesToGlobal(double x, double y,
			std::vector<double> carCoords);
	static MapTrajectory convertCoordinatesToGlobal(std::vector<double> x,
			std::vector<double> y, std::vector<double> carCoords);
	static double norm(double x, double y) {
		return sqrt(pow(x,2)+pow(y,2));
	}


	static double deg2rad(double x) { return x * M_PI / 180; }
	static double rad2deg(double x) { return x * 180 / M_PI; }
	static double getOffAngle(double s, double theta, std::vector<double> maps_s,
			std::vector<double> maps_x, std::vector<double> maps_y);
	static void printVector(std::string prePrint, std::vector<double> vec);

};

#endif /* SRC_HELPER_H_ */
