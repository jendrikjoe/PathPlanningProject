/*
 * SpeedController.cpp
 *
 *  Created on: Jul 20, 2017
 *      Author: jjordening
 */

#include "SpeedController.h"
#include "Helper.h"
#include <math.h>
#include <vector>
#include <iostream>
#define DEBUG false

//The time differnec in seconds between two steps.
const double SpeedController::stepWidth = 0.02;
SpeedController::~SpeedController() {}

/**
 * This function returns a trajectory for the initial case in the simulator
 * @param trajectory - the path which shall be executed
 * @param targetSpeed - the speed with which the path shall be executed
 * @param currentSpeeed - the car's current speed
 * @param carCoords - the car's coordinates
 * @return mapTrajectory - the trajectory which can be send to the simulator
 */
MapTrajectory SpeedController::controlInitial(Trajectory trajectory,
		double targetSpeed, double currentSpeed, std::vector<double> & carCoords) {
	double x = 0;
	double y = 0;
	double dist = 0;
	std::vector<double> localX = std::vector<double>();
	std::vector<double> localY = std::vector<double>();
	localX.push_back(x);
	localY.push_back(y);
	double speed = currentSpeed;
	speed = speed < 1? 1 : speed;
	std::vector<double> speedVec = std::vector<double>();
	std::vector<double> speedPosVec = std::vector<double>();
	double maxLen = trajectory.getMax()<targetSpeed?trajectory.getMax():targetSpeed;

	double endSpeed = targetSpeed<currentSpeed+maxLen?targetSpeed:currentSpeed+maxLen;
	speedVec.push_back(speed);
	speedVec.push_back(speed + (endSpeed-currentSpeed)/2);
	speedVec.push_back(endSpeed);
	speedVec.push_back(endSpeed);
	speedPosVec.push_back(0);
	speedPosVec.push_back(trajectory.getMax()/3.);
	speedPosVec.push_back(trajectory.getMax()*2./3.);
	speedPosVec.push_back(trajectory.getMax());
	if(DEBUG) Helper::printVector("speedVec: ", speedVec);
	if(DEBUG) Helper::printVector("speedPosVec: ", speedPosVec);
	tk::spline speedSpline;
	speedSpline.set_points(speedPosVec, speedVec);

	for (int i = 0; i < 30; i++) {
		dist += speed * stepWidth;
		y = trajectory.getSpline()(dist);

		if(DEBUG and localX.size()>0) std::cout << "Speed: " << speed << " Dist: " << dist
				<< " Real Speed: " << Helper::norm((dist-localX[localX.size()-1])/.02, (y-localY[localY.size()-1])/.02) << std::endl;
		localX.push_back(dist);
		localY.push_back(y);
		speed = speedSpline(dist);
	}
	for (int i = 0; i < 30; i++) {
		dist += speed * stepWidth;
		y = trajectory.getSpline()(dist);

		if (DEBUG and localX.size() > 0)
			std::cout << "Speed: " << speed << " Dist: " << dist
					<< " Real Speed: "
					<< Helper::norm((dist - localX[localX.size() - 1]) / .02,
							(y - localY[localY.size() - 1]) / .02) << std::endl;
		localX.push_back(dist);
		localY.push_back(y);
		speed = speedSpline(dist);
	}
	if(DEBUG) Helper::printVector("speed X: ", localX);
	if(DEBUG) Helper::printVector("speed Y: ", localY);
	return Helper::convertCoordinatesToGlobal(localX, localY, carCoords);
}

/**
 * This function returns a trajectory for simulator
 * @param trajectory - the path which shall be executed
 * @param targetSpeed - the speed with which the path shall be executed
 * @param currentSpeeed - the car's current speed
 * @param carCoords - the car's coordinates
 * @return mapTrajectory - the trajectory which can be send to the simulator
 */
MapTrajectory SpeedController::control(Trajectory trajectory,
		double targetSpeed, double currentSpeed, std::vector<double> & carCoords,
		MapTrajectory previousLocal) {
	double x = 0;
	double y = 0;
	double dist = 0;
	std::vector<double> localX = std::vector<double>();
	std::vector<double> localY = std::vector<double>();
	localX.push_back(x);
	localY.push_back(y);
	double speed = currentSpeed;
	speed = speed < 1? 1 : speed;
	std::vector<double> speedVec = std::vector<double>();
	std::vector<double> speedPosVec = std::vector<double>();
	double maxLen = trajectory.getMax()<targetSpeed?trajectory.getMax():targetSpeed;

	double endSpeed = targetSpeed<currentSpeed+maxLen?targetSpeed:currentSpeed+maxLen;
	speedVec.push_back(speed);
	speedVec.push_back(speed + (endSpeed-currentSpeed)/2);
	speedVec.push_back(endSpeed);
	speedVec.push_back(endSpeed);
	speedPosVec.push_back(0);
	speedPosVec.push_back(trajectory.getMax()/3.);
	speedPosVec.push_back(trajectory.getMax()*2./3.);
	speedPosVec.push_back(trajectory.getMax());
	if(DEBUG) Helper::printVector("speedVec: ", speedVec);
	if(DEBUG) Helper::printVector("speedPosVec: ", speedPosVec);
	tk::spline speedSpline;
	speedSpline.set_points(speedPosVec, speedVec);
	for (int i=0; i<12; i++){
		localX.push_back(previousLocal.getX()[i]);
		dist = previousLocal.getX()[i];
		y = trajectory.getSpline()(dist);
		localY.push_back((1-getWeighting(i, 11)) * previousLocal.getY()[i] +
				getWeighting(i, 11) * y);
	}
	if(localY[1]*localY[2] < 0) localY[1] = 0;
	speed = speedSpline(dist);
	for (int i = 12; i < 50; i++) {
		dist += speed * stepWidth;
		y = trajectory.getSpline()(dist);

		if(DEBUG and localX.size()>0) std::cout << "Speed: " << speed << " Dist: " << dist
				<< " Real Speed: " << Helper::norm((dist-localX[localX.size()-1])/.02, (y-localY[localY.size()-1])/.02) << std::endl;
		localX.push_back(dist);
		localY.push_back(y);
	}
	if(DEBUG) Helper::printVector("speed X: ", localX);
	if(DEBUG) Helper::printVector("speed Y: ", localY);
	return Helper::convertCoordinatesToGlobal(localX, localY, carCoords);
}

/**
 * This function returns a weighting for transitions between
 * trajectories
 * @param val - current value of transition
 * @param max - maximum value of the transition
 * @return weighting
 */
double SpeedController::getWeighting(double val, double max) {
	return 1./(1.+exp(-1.*(val-max/2)));
}
