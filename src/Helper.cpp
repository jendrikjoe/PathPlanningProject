/*
 * Helper.cpp
 *
 *  Created on: Jul 19, 2017
 *      Author: jjordening
 */

#include "Helper.h"
#include <iostream>
#include "Trajectory.h"


/**
 * This function gives the next way point the car has to pass
 * @param x - the x Coordinate to which we want to know the next waypoint
 * @param y - the y Coordinate to which we want to know the next waypoint
 * @param theta - the heading in which we are looking
 * @param maps_x - the x coordinates of the waypoints
 * @param maps_y - the y coordinates of the waypoints
 * @return wp - the next waypoint
 */
int Helper::NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y)
{

	int closestWp = closestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWp];
	double map_y = maps_y[closestWp];

	double heading = atan2( (map_y-y),(map_x-x) );
	double angle = fabs(theta-heading);

	if(angle > M_PI/4)
	{
		closestWp++;
	}

	return closestWp;

}

/**
 * This function gives the spline going throught the given x and y values
 * @param x - the x coordinates for the spline
 * @param y - the y coordinates for the spline
 * @return spline - the spline containing the points
 */
tk::spline Helper::getSplineCurve(std::vector<double> &x, std::vector<double> &y) {
	assert(x.size() == y.size());
	tk::spline spline;
	spline.set_points(x, y);
	return spline;
}

/**
 * This function gives the spline going throught the given x and y values
 * @param xy - the xy coordinates for the spline
 * @return spline - the spline containing the points
 */
tk::spline Helper::getSplineCurve( std::vector<std::vector<double>> &xy) {
	assert(xy.size()==2);
	return getSplineCurve(xy[0], xy[1]);
}


/**
 * This function returns the euclidian distance between two points
 * @param x1 - first point's x Coordinate
 * @param x2 - second point's x Coordinate
 * @param y1 - first point's y Coordinate
 * @param y2 - second point's y Coordinate
 * @return distance between the two points
 */
double Helper::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/**
 * This function gives the closest way point to a given point
 * @param x - the x Coordinate to which we want to know the next waypoint
 * @param y - the y Coordinate to which we want to know the next waypoint
 * @param maps_x - the x coordinates of the waypoints
 * @param maps_y - the y coordinates of the waypoints
 * @return wp - the closest waypoint
 */
int Helper::closestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;
	for(unsigned int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

/**
 * This function converts sensor data from global
 * coordinates to the car's local reference frame
 * @param sensorFusion - The sensor data which shall be converted
 * 		to local coordinates
 * @param carCoords - The car's coordinates
 * @return localFusion - The sensor data in the car's reference frame
 */
std::vector<SensorData> Helper::convertCoordinatesToLocal(std::vector<SensorData> sensorFusion,
		std::vector<double> carCoords) {
	std::vector<SensorData> localFusion = std::vector<SensorData>();
	for(int i = 0; i<sensorFusion.size(); i++) {
		SensorData data = sensorFusion[i];
		std::vector<double> res = convertCoordinatesToLocal(data.getX(), data.getY(), carCoords);
		localFusion.push_back(SensorData(data.getId(), res[0], res[1], data.getVx(),data.getVy(), data.getS(),
				data.getD()));
	}
	return localFusion;
}

/**
 * This function converts from global coordinates to
 * the car's local reference frame
 * @param x - The x coordinate which shall be transformed
 * @param y - The y coordinate which shall be transformed
 * @param carCoords - The car's coordinates
 * @return xy - The coordinates in the car's local reference frame
 */
std::vector<double> Helper::convertCoordinatesToLocal(double x, double y, std::vector<double> carCoords) {
	double shiftedX = x-carCoords[0];
	double shiftedY = y-carCoords[1];
	double rotatedX = shiftedX*cos(carCoords[2])+shiftedY*sin(carCoords[2]);
	double rotatedY = -shiftedX*sin(carCoords[2])+shiftedY*cos(carCoords[2]);
	/*std::cout << " shiftedX: " << shiftedX<< " shiftedY: " << shiftedY
			<< " rotatedX: " << rotatedX<< " rotatedY: " << rotatedY << std::endl;*/
	assert(abs(sqrt(pow(shiftedX,2) + pow(shiftedY,2)) - sqrt(pow(rotatedX,2) + pow(rotatedY,2))) < 0.0001);
	return {rotatedX, rotatedY};
}

/**
 * This function converts from global coordinates to
 * the car's local reference frame
 * @param x - The x coordinates which shall be transformed
 * @param y - The y coordinates which shall be transformed
 * @param carCoords - The car's coordinates
 * @return xy - The coordinates in the car's local reference frame
 */
MapTrajectory Helper::convertCoordinatesToLocal(std::vector<double> x, std::vector<double> y,
		std::vector<double> carCoords) {
	assert(x.size() == y.size());
	std::vector<double> newX = std::vector<double>();
	std::vector<double> newY = std::vector<double>();
	for (int i = 0; i<x.size(); i++) {
		std::vector<double> convertedCoords = convertCoordinatesToLocal(x[i], y[i], carCoords);
		newX.push_back(convertedCoords[0]);
		newY.push_back(convertedCoords[1]);
	}
	return MapTrajectory(newX, newY);
}

/**
 * This function converts from local coordinates to
 * the global reference frame
 * @param x - The x coordinate which shall be transformed
 * @param y - The y coordinate which shall be transformed
 * @param carCoords - The car's coordinates
 * @return xy - The coordinates in the global reference frame
 */
std::vector<double> Helper::convertCoordinatesToGlobal(double x, double y, std::vector<double> carCoords) {
	double rotatedX = x*cos(carCoords[2])-y*sin(carCoords[2]);
	double rotatedY = x*sin(carCoords[2])+y*cos(carCoords[2]);
	assert(abs(sqrt(pow(rotatedX,2) + pow(rotatedY,2)) - sqrt(pow(x,2) + pow(y,2))) < 0.0001);
	return {rotatedX+carCoords[0], rotatedY+carCoords[1]};
}

/**
 * This function converts from local coordinates to
 * the global reference frame
 * @param x - The x coordinates which shall be transformed
 * @param y - The y coordinates which shall be transformed
 * @param carCoords - The car's coordinates
 * @return xy - The coordinates in the global reference frame
 */
MapTrajectory Helper::convertCoordinatesToGlobal(std::vector<double> x, std::vector<double> y,
		std::vector<double> carCoords) {
	assert(x.size() == y.size());
	std::vector<double> newX = std::vector<double>();
	std::vector<double> newY = std::vector<double>();
	for (int i = 0; i<x.size(); i++) {
		std::vector<double> convertedCoords = convertCoordinatesToGlobal(x[i], y[i], carCoords);
		newX.push_back(convertedCoords[0]);
		newY.push_back(convertedCoords[1]);
	}
	return {newX, newY};
}

/**
 * This function prints a vector to the standard output
 * @param prePrint - string which shall be printed in front of the vector
 * @param vec - The vector whcih shall be printed
 */
void Helper::printVector(std::string prePrint, std::vector<double> vec) {
	std::cout << prePrint;
	for(int i=0; i<vec.size(); i++) {
		std::cout << vec[i] << ",";
	}
	std::cout << std::endl;
}

