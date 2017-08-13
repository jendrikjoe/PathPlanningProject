/*
 * BehaviourPlanner.h
 *
 *  Created on: Jul 19, 2017
 *      Author: jjordening
 */

#ifndef SRC_BEHAVIOURPLANNER_H_
#define SRC_BEHAVIOURPLANNER_H_

#include "BehaviourCommand.h"
#include "SensorData.h"
#include <vector>
#include "Trajectory.h"
#include "Lanes.h"
#include <map>
#include <chrono>

/**
 * This class is allowing to plan the future lane and speed
 * for a car on a highway.
 */
class BehaviourPlanner {

public:
	BehaviourPlanner();
	virtual ~BehaviourPlanner();
	BehaviourCommand getNewState(std::vector<SensorData> sensorFusion, std::vector<double> carCoords,
			double currentSpeed);
private:
	std::map<Lanes, std::vector<SensorData>> splitSensorDataByLane(std::vector<SensorData> sensorFusion);
	std::map<double, Lanes> getCostOptions(std::map<Lanes, std::vector<SensorData>> laneSensorData,
			Lanes currentLane, double currentSpeed);
	Lanes selectLane(std::map<double, Lanes> costOptions,
			std::map<Lanes, std::vector<SensorData>> laneSensorData,
			Lanes currentLane, double currentSpeed);
	double selectSpeed(std::map<Lanes, std::vector<SensorData>> laneSensorData,
			double currentD, Lanes selectedLane, double currentSpeed);
	Lanes lastDecision = Lanes::LAST;
	std::chrono::time_point<std::chrono::system_clock> previousDecision = std::chrono::system_clock::now();

};

#endif /* SRC_BEHAVIOURPLANNER_H_ */
