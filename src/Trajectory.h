/*
 * FrenetCoordinates.h
 *
 *  Created on: Jul 19, 2017
 *      Author: jjordening
 */

#ifndef SRC_TRAJECTORY_H_
#define SRC_TRAJECTORY_H_
#include <vector>
#include <string>
#include <assert.h>
#include <math.h>
#include "spline.h"
//#define NDEBUG

/**
 * This class serves as a container for a planned path.
 */
class Trajectory {
private:
	tk::spline spline;
	double min;
	double max;

public:
	Trajectory(){}

	Trajectory(tk::spline spline, double min, double max) : spline(spline), min(min), max(max) {}

	Trajectory& operator=( const Trajectory& other) {
		spline = other.spline;
		min = other.min;
		max = other.max;
		return *this;
	};

	const tk::spline& getSpline() const {
		return spline;
	}

	double getMax() const {
		return max;
	}

	double getMin() const {
		return min;
	}
};


#endif /* SRC_FRENETTRAJECTORY_H_ */
