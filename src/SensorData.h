/*
 * SensorData.h
 *
 *  Created on: Aug 9, 2017
 *      Author: jjordening
 */

#ifndef SRC_SENSORDATA_H_
#define SRC_SENSORDATA_H_
#include <math.h>
#include <iostream>

/**
 * This class serves as a container for sensor data
 */
class SensorData {
public:
	SensorData(int id, double x, double y, double vx, double vy, double s,
			double d) :
			id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d) {
		speed = sqrt(pow(vx, 2) + pow(vy, 2));
	}
	SensorData() :
			id(-1), x(-1), y(-1), vx(-1), vy(-1), s(-1), d(-1) {
		speed = -1;
	}

	double getD() const {
		return d;
	}

	int getId() const {
		return id;
	}

	double getS() const {
		return s;
	}

	double getVx() const {
		return vx;
	}

	double getVy() const {
		return vy;
	}

	double getX() const {
		return x;
	}

	double getY() const {
		return y;
	}
	double getSpeed() const {
		return speed;
	}
	void print() const{
		std::cout << "[" << x << "," << y << "," << vx << "," << vy << "," << s << "," << d << "]" << std::endl;
	}
	~SensorData() {}

private:
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	double speed;


};

#endif /* SRC_SENSORDATA_H_ */
