/*
 * SpeedController.h
 *
 *  Created on: Jul 20, 2017
 *      Author: jjordening
 */

#ifndef SRC_SPEEDCONTROLLER_H_
#define SRC_SPEEDCONTROLLER_H_

#include "MapTrajectory.h"
#include "Trajectory.h"
#include <chrono>
#include <vector>
#include "spline.h"

/**
 * This class reflects a speed controller, which produces
 * trajectories reflecting the right speed in the simulator.
 */
class SpeedController {
public:
	SpeedController(double jerkLimit, double accLimit) : jerkLimit(jerkLimit), accLimit(accLimit) {};
	virtual ~SpeedController();
	MapTrajectory controlInitial(Trajectory trajectory,
			double targetSpeed, double currentSpeed, std::vector<double> & carCoords);
	MapTrajectory control(Trajectory trajectory,
			double targetSpeed, double currentSpeed, std::vector<double> & carCoords,
			MapTrajectory previousLocal);

private:
	static double getWeighting(double val, double max);
	double jerkLimit;
	double accLimit;
	double previousSpeed = 0;
	static const double stepWidth;
	std::chrono::time_point<std::chrono::system_clock> previousTimestamp = std::chrono::system_clock::now();
};

#endif /* SRC_SPEEDCONTROLLER_H_ */
