/*
 * BehaviourPlanner.cpp
 *
 *  Created on: Jul 19, 2017
 *      Author: jjordening
 */

#include "BehaviourPlanner.h"
#include "Helper.h"
#include "Lanes.h"
#define MAX_SPEED 21.5
#include <iostream>
#include <map>

/**
 * The default constructor for the behaviour planner
 */
BehaviourPlanner::BehaviourPlanner() {}

/**
 * This function gives the new commands the car shall execute
 * @param sensorFusion - the sensor data localising the other cars
 * @param carCoords - the car's coordinates
 * @param currentSpeed - the car's current speed
 * @return BehaviourCommand - a task the car shall execute
 */
BehaviourCommand BehaviourPlanner::getNewState(std::vector<SensorData> sensorFusion, std::vector<double> carCoords,
		double currentSpeed) {
	Lanes currentLane = LEFT_LANE;
	if(carCoords[4] > 4) currentLane = MIDDLE_LANE;
	if(carCoords[4] > 8) currentLane = RIGHT_LANE;
	sensorFusion = Helper::convertCoordinatesToLocal(sensorFusion, carCoords);
	for (int i = 0; i < sensorFusion.size(); i++) {
		auto data = sensorFusion[i];
	}
	std::map<Lanes, std::vector<SensorData>> laneSensorData = splitSensorDataByLane(sensorFusion);
	std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
	Lanes selectedLane;
	std::chrono::duration<double> elapsedSeconds = now-previousDecision;
	//check if the time elapsed since the last decision is longer than 3 seconds
	if(elapsedSeconds.count() > 3) {

		std::map<double, Lanes> costOptions = getCostOptions(laneSensorData,
				currentLane, currentSpeed);
		selectedLane = selectLane(costOptions, laneSensorData, currentLane, currentSpeed);
		lastDecision = selectedLane;
		previousDecision = now;
	//otherwise stick with the old decision
	} else {
		selectedLane = lastDecision;
	}
	double speedCommand = selectSpeed(laneSensorData, carCoords[4], selectedLane, currentSpeed);
	return BehaviourCommand(selectedLane, speedCommand);

}

/**
 * This function splits the sensor data handed, such that the
 * cars are assigned to each lane. The cars are hereby assigned
 * to one, if they are driving in the middle and to two, if
 * they are executing a lane change.
 * @param sensorFusion - the sensor data localising the other cars
 * @return BehaviourCommand - the sensor date localising other cars
 * 			in their corresponding lanes
 */
std::map<Lanes, std::vector<SensorData>> BehaviourPlanner::splitSensorDataByLane(std::vector<SensorData> sensorFusion) {
	std::map<Lanes, std::vector<SensorData>> laneSensorData = std::map<Lanes, std::vector<SensorData>>();
	std::map<double, Lanes> costs = std::map<double, Lanes>();

	for(int i = LEFT_LANE; i<LAST; i++) {
		laneSensorData.insert(std::pair<Lanes, std::vector<SensorData>>(static_cast<Lanes>(i),
				std::vector<SensorData>()));
	}
	for(int i=0; i<sensorFusion.size(); i++) {
		SensorData data = sensorFusion[i];
		if(sqrt(pow(data.getX(),2)+pow(data.getY(),2)) > 300) continue;
		if(data.getD() < 3) laneSensorData.at(LEFT_LANE).push_back(data);
		// Car changing from left to middle lane or vice verse
		else if(data.getD() < 5) {
			laneSensorData.at(LEFT_LANE).push_back(data);
			laneSensorData.at(MIDDLE_LANE).push_back(data);
		}
		else if(data.getD() < 7) laneSensorData.at(MIDDLE_LANE).push_back(data);
		//car changing from middle to right line or vice verse
		else if (data.getD() < 9){
			laneSensorData.at(MIDDLE_LANE).push_back(data);
			laneSensorData.at(RIGHT_LANE).push_back(data);
		}
		else laneSensorData.at(RIGHT_LANE).push_back(data);
	}
	return laneSensorData;
}

/**
 * This function determines the cost for each of the lanes
 * @param laneSensorData - the sensor data localising the other cars
 * 		in their corresponding lanes
 * @param currentLane - the lane the car is currently in
 * @param currentSpeed - the car's current speed
 * @return costOptions - a map assigning a cost factor to each of the lanes.
 */
std::map<double, Lanes> BehaviourPlanner::getCostOptions(
		std::map<Lanes, std::vector<SensorData>> laneSensorData,
		Lanes currentLane, double currentSpeed) {
	double speedCommand = 0;
	double minCost = 1e7;
	std::map<double, Lanes> costOptions = std::map<double, Lanes>();
	for (std::map<Lanes, std::vector<SensorData>>::iterator it =
			laneSensorData.begin(); it != laneSensorData.end(); ++it) {
		std::vector<SensorData> laneData = it->second;
		double distanceForward = 100.;
		double distanceBackward = 100.;
		SensorData carInFront;
		SensorData carInBack;
		for (int i = 0; i < laneData.size(); i++) {
			SensorData data = laneData[i];
			double distance = sqrt(pow(data.getX(), 2) + pow(data.getY(), 2));
			if (data.getX() > 0) {
				if (distance < distanceForward) {
					distanceForward = distance;
					carInFront = data;
				}
			} else {
				if (distance < distanceBackward) {
					distanceBackward = distance;
					carInBack = data;
				}
			}
		}
		double cost = 0;
		//cost += 12 * it->first;
		if (it->first != currentLane) {
			cost += 10;
			cost += abs(it->first - currentLane);
			if (distanceBackward < 5)
				cost += 100000.;
			if (distanceBackward < 10 && carInBack.getSpeed() - currentSpeed > 1)
				cost += 100000.;
		}
		if (distanceForward < 10)
			cost += 100000.;
		cost += (100 - distanceForward);
		if (distanceForward != 100.)
			cost += 10 * (fmax(MAX_SPEED - carInFront.getSpeed(), 0));
		std::vector<double> laneCommands = std::vector<double>();
		costOptions.insert(std::pair<double, Lanes>(cost, it->first));
	}
	return costOptions;
}

/**
 * This function selects a lane the car shall drive to.
 * @param costOptions - a map assigning a cost factor to each of the lanes.
 * @param laneSensorData - the sensor data localising the other cars
 * 		in their corresponding lanes
 * @param currentLane - the lane the car is currently in
 * @param currentSpeed - the car's current speed
 * @return targetLane - the lane the car shall drive to
 */
Lanes BehaviourPlanner::selectLane(std::map<double, Lanes>  costOptions,
		std::map<Lanes, std::vector<SensorData>> laneSensorData,
		Lanes currentLane, double currentSpeed) {
	for (std::map<double, Lanes>::iterator it = costOptions.begin();
			it != costOptions.end(); ++it) {
		if (abs(it->second - currentLane) > 1) {
			for (std::map<double, Lanes>::iterator it2 =
					costOptions.begin(); it2 != costOptions.end(); ++it2) {
				if (it2->second != MIDDLE_LANE)
					continue;
				if (it2->first < 10000.) {
					std::vector<SensorData> sensorDataCrossedLane =
							laneSensorData.at(MIDDLE_LANE);
					double distanceForward = 100.;
					double distanceBackward = 100.;
					SensorData carInFront;
					SensorData carInBack;
					for (int i = 0; i < sensorDataCrossedLane.size(); i++) {
						SensorData data = sensorDataCrossedLane[i];
						double distance = sqrt(
								pow(data.getX(), 2) + pow(data.getY(), 2));
						if (data.getX() > 0) {
							if (distance < distanceForward) {
								distanceForward = distance;
								carInFront = data;
							}
						} else {
							if (distance < distanceBackward) {
								distanceBackward = distance;
								carInBack = data;
							}
						}
					}
					if (distanceForward < 25 && carInFront.getSpeed() - currentSpeed < -5) continue;
					if (distanceBackward < 15 && carInBack.getSpeed() - currentSpeed > 2) continue;
					if (distanceForward < 10) continue;
					if (distanceBackward < 10) continue;
					return  it->second;
				}
			}
		} else {
			return it->second;
		}
	}
	return currentLane;
}

/**
 * This function selects the speed the car shall drive.
 * @param laneSensorData - the sensor data localising the other cars
 * 		in their corresponding lanes
 * @param currentLane - the lane the car is currently in
 * @param selectedLane - the lane the car will drive to
 * @param currentSpeed - the car's current speed
 * @return targetSpeed - the speed the car shall drive
 */
double BehaviourPlanner::selectSpeed(std::map<Lanes, std::vector<SensorData>> laneSensorData,
		double currentD, Lanes selectedLane, double currentSpeed) {
	double speedCommand = 0;
	if (4*selectedLane +2 - currentD > 2) {
		std::vector<SensorData> sensorDataCrossedLane = laneSensorData.at(
				static_cast<Lanes>(1));
		double distanceForward = 100.;
		double distanceBackward = 100.;
		SensorData carInFront;
		SensorData carInBack;
		for (int i = 0; i < sensorDataCrossedLane.size(); i++) {
			SensorData data = sensorDataCrossedLane[i];
			double distance = sqrt(pow(data.getX(), 2) + pow(data.getY(), 2));
			if (data.getX() > 0) {
				if (distance < distanceForward) {
					distanceForward = distance;
					carInFront = data;
				}
			} else {
				if (distance < distanceBackward) {
					distanceBackward = distance;
					carInBack = data;
				}
			}
		}
		if(distanceForward > 25) return MAX_SPEED;
		else if(distanceForward > 15) return carInFront.getSpeed() + 2;
		else if(distanceForward > 5) return carInFront.getSpeed() -2;
		else return carInFront.getSpeed() - 5;
	}
	double distanceForward = 100.;
	SensorData carInFront;
	SensorData carInBack;
	std::vector<SensorData> laneData = laneSensorData.at(selectedLane);
	for (int i = 0; i < laneData.size(); i++) {
		SensorData data = laneData[i];
		double distance = sqrt(pow(data.getX(), 2) + pow(data.getY(), 2));
		if (data.getX() > 0) {
			if (distance < distanceForward) {
				distanceForward = distance;
				carInFront = data;
			}
		}
	}
	if (distanceForward < 5)
		speedCommand = fmin(MAX_SPEED, carInFront.getSpeed() - 5);
	else if (distanceForward < 10)
		speedCommand = fmin(MAX_SPEED, carInFront.getSpeed() - 2);
	else if (distanceForward < 20)
		speedCommand = fmin(MAX_SPEED, carInFront.getSpeed() + .5);
	else if (distanceForward < 30)
		speedCommand = fmin(MAX_SPEED, carInFront.getSpeed() + 2.);
	else
		speedCommand = MAX_SPEED;

	return speedCommand;


}

BehaviourPlanner::~BehaviourPlanner() {
	// TODO Auto-generated destructor stub
}

