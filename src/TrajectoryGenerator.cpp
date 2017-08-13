/*
2 * TrajectoryGenerator.cpp
 *
 *  Created on: Jul 19, 2017
 *      Author: jjordening
 */

#include "TrajectoryGenerator.h"
#include <math.h>
#include <assert.h>
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
#include "Helper.h"
#define DEBUG false

TrajectoryGenerator::~TrajectoryGenerator() {
}


/**
 * This function builds a path for the car
 * @param targetLane - the lane in which the car shall end up in
 * @param previousPathLocal - the previously path of the car in the
 * 			car's local reference frame
 * @param carCoords - the car's coordinates
 * @param currentSpeed - the car's current speed
 * @return trajectory - the path, which the car shall take
 */
Trajectory TrajectoryGenerator::getTrajectory(Lanes targetLane, MapTrajectory & previousPathLocal,
			std::vector<double> & carCoords, double currentSpeed) {

	double targetD = 6;
	if(targetLane == Lanes::LEFT_LANE) targetD = 2;
	else if(targetLane == Lanes::RIGHT_LANE) targetD = 10;
	double maxD = currentSpeed/8.;
	double dir = (targetD-carCoords[4])/fabs(targetD-carCoords[4]);
	targetD = abs(targetD-carCoords[4])>maxD? carCoords[4]+dir*maxD:targetD;

	if(DEBUG) Helper::printVector("localX: ", previousPathLocal.getX());
	if(DEBUG) Helper::printVector("localY: ", previousPathLocal.getY());

	tk::spline localSpline;
	localSpline.set_points(previousPathLocal.getX(), previousPathLocal.getY());

	std::vector<std::vector<double>> lineTrajectory = getLineTrajectory(carCoords, targetD, maps_x,
			maps_y, maps_dx, maps_dy);
	if(DEBUG) Helper::printVector("lineX: ", lineTrajectory[0]);
	if(DEBUG) Helper::printVector("lineY: ", lineTrajectory[1]);


	tk::spline localTarget;
	localTarget.set_points(lineTrajectory[0], lineTrajectory[1]);
	if(DEBUG) std::cout << "previous max X: " << previousPathLocal.getX()[previousPathLocal.getX().size()-1] << std::endl;
	double pointDistance = previousPathLocal.getX()[previousPathLocal.getX().size()-1]/20.;
	if(DEBUG) std::cout << "Point Distance: " << pointDistance << std::endl;
	int transitionStart = 10;
	int transitionEnd = 40;
	int numberOfPoints = 50;
	if(DEBUG) std::cout << "Transition Start: " << transitionStart << " Transition End: " << transitionEnd << std::endl;

	std::vector<double> mergedX = std::vector<double>();
	std::vector<double> mergedY = std::vector<double>();

	for(int i = 0; i<numberOfPoints; i++) {
		if(i < transitionStart) {
			mergedX.push_back(i*pointDistance);
			mergedY.push_back(localSpline(i*pointDistance));
		} else if(i<transitionEnd) {
			continue;
		} else {
			mergedX.push_back(i*pointDistance);
			mergedY.push_back(localTarget(i*pointDistance));
		}
	}
	if(DEBUG) Helper::printVector("mergedX: ", mergedX);
	if(DEBUG) Helper::printVector("mergedY: ", mergedY);
	tk::spline trajectory;
	trajectory.set_points(mergedX, mergedY);
	return Trajectory(trajectory, 0, (numberOfPoints-1)*pointDistance);
}

/**
 * This function builds an initial path for the start car
 * @param targetLane - the lane in which the car shall end up in
 * @param carCoords - the car's coordinates
 * @return trajectory - the path, which the car shall take
 */
Trajectory TrajectoryGenerator::getInitialTrajectory(Lanes targetLane,
			std::vector<double> & carCoords) {
	double targetD = 6;

	std::vector<std::vector<double>> lineTrajectory = getLineTrajectory(carCoords, targetD, maps_x,
			maps_y, maps_dx, maps_dy);

	tk::spline localTarget;
	localTarget.set_points(lineTrajectory[0], lineTrajectory[1]);

	std::vector<double> mergedX = std::vector<double>();
	std::vector<double> mergedY = std::vector<double>();
	mergedX.push_back(0);
	mergedY.push_back(0);
	for(int i = 1; i<10; i++) {
		mergedX.push_back(i*5);
		mergedY.push_back(localTarget(i*5));
	}

	if(DEBUG) Helper::printVector("mergedX: ", mergedX);
	if(DEBUG) Helper::printVector("mergedY: ", mergedY);
	tk::spline trajectory;
	trajectory.set_points(mergedX, mergedY);
	return Trajectory(trajectory, 0, 100);
}

/**
 * This function gives the target trajectory for a certain d
 * @param carCoors - the car's coordinates
 * @param d - the target d value
 * @param maps_x - the x coordinates of the waypoints
 * @param maps_y - the y coordinates of the waypoints
 * @param maps_dx - the dx coordinates of the waypoints
 * @param maps_dy - the dy coordinates of the waypoints
 * @return wp - the next waypoint
 */
std::vector<std::vector<double>> TrajectoryGenerator::getLineTrajectory(std::vector<double> carCoords, double d,
		std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_dx, std::vector<double> maps_dy) {
	int cWp = Helper::closestWaypoint(carCoords[0], carCoords[1], maps_x, maps_y);
	std::vector<double> localX = std::vector<double>();
	std::vector<double> localY = std::vector<double>();
	for (int i = -2; i<10; i++) {
		int wpWithoutModulus = cWp+i;
		if(wpWithoutModulus < 0) wpWithoutModulus = wpWithoutModulus+maps_x.size();
		int wp = (wpWithoutModulus)%maps_x.size();
		std::vector<double> convertedCoords = Helper::convertCoordinatesToLocal(maps_x[wp] + d*maps_dx[wp],
				maps_y[wp] + d*maps_dy[wp], carCoords);
		localX.push_back(convertedCoords[0]);
		localY.push_back(convertedCoords[1]);
	}
	return {localX, localY};
}
