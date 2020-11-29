#include <mrm-8x8a.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-col-can.h>
#include <mrm-imu.h>
#include <mrm-ir-finder3.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-mot4x10.h>
#include "mrm-robot-min.h"
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>


/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotMin::RobotMin(char name[]) : Robot(name) {
	// MotorGroup class drives the motors.
	// 2nd, 4th, 6th, and 8th parameters are output connectors of the controller (0 - 3, meaning 1 - 4. connector). 2nd one must be connected to LB (Left-Back) motor,
	// 4th to LF (Left-Front), 6th to RF (Right-Front), and 8th to RB (Right-Back). Therefore, You can connect motors freely, but have to
	// adjust the parameters here. In this example output (connector) 3 is LB, etc.
	motorGroup = new MotorGroupDifferential(this, mrm_mot4x3_6can, 3, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 0);

	// Depending on your wiring, it may be necessary to spin some motors in the other direction. In this example, no change needed,
	// but uncommenting the following line will change the direction of the motor 2.
	//mrm_mot4x3_6can->directionChange(2);

	// All the actions will be defined here; the objects will be created.


	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus. For example, ActionWallsTest test is only manu-based, and it is all right. 
	// This test is not supposed to be called in code.


	// Set buttons' actions
	//mrm_8x8a->actionSet(_actionMenuMain, 0); // Stop and display menu
	//mrm_8x8a->actionSet(_actionAny, 1); // Free-defined action.
	//mrm_8x8a->actionSet(_actionCANBusStress, 2); // Starts stress test.
	// Put Your buttons' actions here.

	// Upload custom bitmaps into mrm-8x8a.
	bitmapsSet();
}

/** Custom test. The function will be called many times during the test, till You issue "x" menu-command.
*/
void RobotMin::loop() {

	static int16_t speed = 0;
	static bool up = true;
	static bool even = true;
	if (setup()) { // This part will execute only in the firs run.
		pinMode(27, OUTPUT);
		pinMode(26, INPUT);
		delay(500);
	}

	// mrm-mot4x3.6_can
	motorGroup->go(speed, speed);
	if (up)
		speed++;
	else
		speed--;

	// mrm-mot4x10
	for (uint8_t i = 0; i < 4; i++)
		mrm_mot4x10->speedSet(i, speed);

	// mrm-fets
	if (speed == 120 && up) {
		digitalWrite(27, HIGH);
		delay(150);
		digitalWrite(27, LOW);
	}

	// Change motor direction
	if (up && speed > 125 || !up && speed < -125)
		up = !up;
	
	// mrm-led8x8
	if (even)
		mrm_8x8a->text("a");
	else
		mrm_8x8a->text("b");
	even = !even;

	// mrm-col-can, mrm-ir-finder3, mrm-lid-can-b2, mrm-ref-can, mrm-therm-b-can, mrm-barr2*, mrm-enc
	print("lid:%imm ir:%i refArr:%i th:%ideg barr:%i col:%i, enc:%i, col:%i\n\r", mrm_lid_can_b2->reading(0), mrm_ir_finder3->reading(0), mrm_ref_can->reading(0), mrm_therm_b_can->reading(0),
		analogRead(36), mrm_col_can->reading(0), digitalRead(26), mrm_col_can->reading(0));

	//print("lid:%imm refArr:%i th:%ideg \n\r", mrm_lid_can_b2->reading(0), mrm_ref_can->reading(0), mrm_therm_b_can->reading(0));
	//actionSet(_actionAny);
}
