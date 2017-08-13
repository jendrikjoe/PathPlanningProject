/*
 * TrajectoryGenerator.h
 *
 *  Created on: Jul 19, 2017
 *      Author: jjordening
 */

#ifndef SRC_TRAJECTORYGENERATOR_H_
#define SRC_TRAJECTORYGENERATOR_H_
#include <vector>
#include "MapTrajectory.h"
#include <chrono>
#include "Trajectory.h"
#include "spline.h"
#include "Lanes.h"

/**
 * This class represents a generator for trajectories
 */
class TrajectoryGenerator {
public:
	TrajectoryGenerator(const double trackLength,
			std::vector<double> maps_x, std::vector<double> maps_y,
					std::vector<double> maps_dx, std::vector<double> maps_dy) :
		trackLength(trackLength),maps_x(maps_x), maps_y(maps_y), maps_dx(maps_dx), maps_dy(maps_dy) {};
	virtual ~TrajectoryGenerator();
	Trajectory getTrajectory(Lanes targetLane, MapTrajectory & previousPathLocal,
			std::vector<double> & carCoords, double currentSpeed);
	Trajectory getInitialTrajectory(Lanes state,
				std::vector<double> & carCoords);

private:
	std::vector<std::vector<double>> getLineTrajectory(std::vector<double> carCoords, double d,
			std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_dx, std::vector<double> maps_dy);
	double previousD = -1;
	double previousDDot = -1;
	const double trackLength;
	std::vector<double> maps_x;
	std::vector<double> maps_y;
	std::vector<double> maps_dx;
	std::vector<double> maps_dy;

	std::chrono::time_point<std::chrono::system_clock> previousTimestamp = std::chrono::system_clock::now();

};

#endif /* SRC_TRAJECTORYGENERATOR_H_ */
