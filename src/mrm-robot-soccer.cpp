#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b2.h>
#include <mrm-ir-finder2.h>
#include <mrm-ir-finder3.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include "mrm-robot-soccer.h"

/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotSoccer::RobotSoccer(char name[]) : Robot(name) {
	motorGroup = new MotorGroupStar(this, mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 3);

	pidXY = new Mrm_pid(0.5, 200, 0); // PID controller, regulates motors' speeds for linear motion in the x-y plane
	pidRotation = new Mrm_pid(0.5, 100, 0); // PID controller, regulates rotation around z axis
	actionAdd(new ActionSoccerPlay(this));
	actionCatch = new ActionSoccerCatch(this);
	actionIdle = new ActionSoccerIdle(this);

	mrm_mot4x3_6can->directionChange(0); // Uncomment to change 1st wheel's rotation direction
	mrm_mot4x3_6can->directionChange(1); // Uncomment to change 2nd wheel's rotation direction
	mrm_mot4x3_6can->directionChange(2); // Uncomment to change 3rd wheel's rotation direction
	mrm_mot4x3_6can->directionChange(3); // Uncomment to change 4th wheel's rotation direction

	mrm_8x8a->actionSet(_actionLoop, 1); // Button 1 starts user defined loop() function
}

/** Read barrier
@return - true if interrupted
*/
bool RobotSoccer::barrier() {
	return analogRead(35) < 300; // Adjust this value
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotSoccer::bitmapsSet() {
	uint8_t red[8] = { 0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000 };
	uint8_t green[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
	mrm_8x8a->bitmapCustomStore(red, green, 7);
}

/** Go around the ball and approach it.
*/
void RobotSoccer::catchBall() {
	if (mrm_ir_finder2->anyIRSource()) {

	}
	else
		actionSet(actionIdle);
}

/** Front distance to wall
@return - in mm
*/
uint16_t RobotSoccer::front() {
	return mrm_lid_can_b2->reading(0); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Test - go straight ahead.
*/
void RobotSoccer::goAhead() {
	const uint8_t speed = 40;
	motorGroup->go(speed);
	end();
}

/** No ball detected - return to Your goal.
*/
void RobotSoccer::idle() {
	if (setup())
		headingToMaintain = mrm_imu->heading();
	if (mrm_ir_finder2->anyIRSource() && false)
		actionSet(actionCatch);
	else {
		//print("\n\rError: %i = %i - 300\n\r", (int)(300 - robot->mrm_lid_can_b2->reading(3)), robot->mrm_lid_can_b2->reading(3));
		float errorL = 900 - mrm_lid_can_b2->reading(3);
		float errorR = mrm_lid_can_b2->reading(1) - 900;
		motorGroup->goToEliminateErrors(errorL > errorR ? errorL : errorR, 150 - mrm_lid_can_b2->reading(2), headingToMaintain - mrm_imu->heading(), pidXY, pidRotation, true);
		//delay(500);
	}
}

/** Left distance to wall
@return - in mm
*/
uint16_t RobotSoccer::left() {
	return mrm_lid_can_b2->reading(3); // Correct all sensors so that they return the same value for the same physical distance.
}


/** Custom test.
*/
void RobotSoccer::loop() {
	motorGroup->go(100, 0);
	//static int initialDirection;
	//if (setup()) {
	//	initialDirection = mrm_imu->heading();
	//}

	//int rotationalError = mrm_imu->heading() - initialDirection;

	//int positionError = 0;
	//if (mrm_lid_can_b2->reading(1) > 80)
	//	positionError = 91 - mrm_lid_can_b2->reading(1);
	//else if (mrm_lid_can_b2->reading(3) > 80)
	//	positionError = mrm_lid_can_b2->reading(1) - 91;

	//motorGroup->go(abs(positionError), positionError < 0 ? -90 : 90, rotationalError);
}

/** Starts robot
*/
void RobotSoccer::play() {
	if (motorGroup == NULL) {
		print("Define robot->motorGroupStar first.\n\r");
		return;
	}
	headingToMaintain = mrm_imu->heading();
	actionSet(actionIdle);
}

/** Rear distance to wall
@return - in mm
*/
uint16_t RobotSoccer::rear() {
	return mrm_lid_can_b2->reading(2); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Right distance to wall
@return - in mm
*/
uint16_t RobotSoccer::right() {
	return mrm_lid_can_b2->reading(1); // Correct all sensors so that they return the same value for the same physical distance.
}