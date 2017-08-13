/*
 * MapTrajectory.h
 *
 *  Created on: Jul 19, 2017
 *      Author: jjordening
 */

#ifndef SRC_MAPTRAJECTORY_H_
#define SRC_MAPTRAJECTORY_H_
#include <vector>
#include <string>
#include "spline.h"
#include <assert.h>
//#define NDEBUG

/**
 * This class is a container for an xy-trajectory
 */
class MapTrajectory {
private:
	std::vector<double> xVector;
	std::vector<double> yVector;
public:
	MapTrajectory(){}
	MapTrajectory(std::vector<double> & xVector,
			std::vector<double> & yVector) : xVector(xVector), yVector(yVector){
			assert(xVector.size() == yVector.size());
		};
	MapTrajectory(const MapTrajectory& other) : xVector(other.xVector),
	yVector(other.yVector) {};

	MapTrajectory& operator=( const MapTrajectory& other) {
		xVector = other.xVector;
		yVector = other.yVector;
		return *this;
	};

	const std::vector<double> getX() {
		return xVector;
	};

	const std::vector<double> getY() {
			return yVector;
	};
};


#endif /* SRC_MAPTRAJECTORY_H_ */
