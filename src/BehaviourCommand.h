/*
 * BehaviourCommand.h
 *
 *  Created on: Aug 9, 2017
 *      Author: jjordening
 */

#ifndef SRC_BEHAVIOURCOMMAND_H_
#define SRC_BEHAVIOURCOMMAND_H_

#include "Lanes.h"

class BehaviourCommand {
	/**
	 * This class serves as a container for the output from the
	 * BehaviourPlanner. It contains the target lane and speed.
	 */
public:
	BehaviourCommand(Lanes laneCommand, double speedCommand) :
		laneCommand(laneCommand), speedCommand(speedCommand) {}

	Lanes getLaneCommand() const {
		return laneCommand;
	}

	double getSpeedCommand() const {
		return speedCommand;
	}

private:
	Lanes laneCommand;
	double speedCommand;

};



#endif /* SRC_BEHAVIOURCOMMAND_H_ */
