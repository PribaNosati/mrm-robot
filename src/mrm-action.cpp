#include <mrm-action.h>
#include <mrm-8x8a.h>
#include <mrm-col-b.h>
#include <mrm-col-can.h>
#include <mrm-imu.h>
#include <mrm-node.h>
#include <mrm-ir-finder3.h>
//#include <mrm-ir-finder-can.h>
#include <mrm-ref-can.h>
#include <mrm-servo.h>
#include <mrm-robot.h>
#include <mrm-us-b.h>
#include <mrm-us1.h>

/** Constructor
@param robot - robot
@param shortcut - up-to-3-letter word
@param text - menu entry
@param menuLevel - all the actions with the same menuLevel are displayed. Permitted values are: 0 (in no menu), 1, 2, 4, 8, 16, 32, 64, and 128. 
	A menu-action changes menuLevel to its own, forcing all the actions with this menuLevel to be displayed. "|" lists action in many menus, for example 1 | 8 | 16.
@param boardId - menu only for a specific board
*/
ActionBase::ActionBase(Robot* robot, const char shortcut[4], const char text[20], uint8_t menuLevel, BoardId boardsId) {
	_robot = robot;
	if (shortcut != 0)
		strcpy(_shortcut, shortcut);
	if (text != 0)
		strcpy(_text, text);
	_menuLevel = menuLevel;
	_boardsId = boardsId;
}

void Action8x8Test::perform() { _robot->mrm_8x8a->test(); }
void ActionBluetoothTest::perform() { _robot->bluetoothTest(); }
void ActionCANBusScan::perform() { _robot->devicesScan(true); }
void ActionCANBusSniff::perform() { _robot->canBusSniffToggle(); }
void ActionCANBusStress::perform() { _robot->stressTest(); }
void ActionColorBTest6Colors::perform() { _robot->mrm_col_b->test(false);}
void ActionColorBTestHSV::perform() { _robot->mrm_col_b->test(true); }
void ActionColorIlluminationOff::perform() { _robot->colorIlluminationOff(); }
void ActionColorIlluminationOn::perform() { _robot->colorIlluminationOn(); }
void ActionColorPatternErase::perform() { _robot->colorPatternErase(); }
void ActionColorPatternPrint::perform() { _robot->colorPatternPrint(); }
void ActionColorPatternRecognize::perform() { _robot->colorPatternRecognize(); }
void ActionColorPatternRecord::perform() { _robot->colorPatternRecord(); }
void ActionColorTest6Colors::perform() { _robot->mrm_col_can->test(false);}
void ActionColorTestHSV::perform() { _robot->mrm_col_can->test(true); }
void ActionDeviceIdChange::perform() { _robot->canIdChange(); }
void ActionFirmware::perform() { _robot->firmwarePrint(); }
void ActionFPS::perform() { _robot->fpsPrint(); }
void ActionGoAhead::perform() { _robot->goAhead(); }
void ActionI2CTest::perform() { _robot->i2cTest(); }
void ActionIRFinderTest::perform() { _robot->mrm_ir_finder3->test(); }
void ActionIRFinderCanTest::perform() { _robot->irFinder3Test(); }
void ActionIRFinderCanTestCalculated::perform() { _robot->irFinder3TestCalculated(); }
void ActionIMUTest::perform() { _robot->mrm_imu->test(); }
void ActionInfo::perform() { _robot->info(); }
void ActionLidarCalibrate::perform() { _robot->lidarCalibrate(); }
void ActionLidar2mTest::perform() { _robot->lidar2mTest(); }
void ActionLidar4mTest::perform() { _robot->lidar4mTest(); }
void ActionLoop::perform() { _robot->loop(); }
void ActionMenuColor::perform() { _robot->menuColor(); }
void ActionMenuColorB::perform() { _robot->menuColor(); }
void ActionMenuMain::perform() { _robot->menuMainAndIdle(); }
void ActionMenuReflectance::perform() { _robot->menuReflectance(); }
void ActionMenuSystem::perform() { _robot->menuSystem(); }
void ActionMotorTest::perform() { _robot->motorTest(); }
void ActionNodeTest::perform() { _robot->nodeTest(); }
void ActionNodeServoTest::perform() { _robot->mrm_node->servoTest();}
//void ActionOscillatorTest::perform() { _robot->oscillatorTest(); }
void ActionReflectanceArrayAnalogTest::perform() { _robot->mrm_ref_can->test(true); }
void ActionReflectanceArrayDigitalTest::perform() { _robot->mrm_ref_can->test(false); }
void ActionReflectanceArrayCalibrate::perform() { _robot->mrm_ref_can->calibrate(); }
void ActionReflectanceArrayCalibrationPrint::perform() { _robot->reflectanceArrayCalibrationPrint(); }
void ActionServoInteractive::perform() { _robot->servoInteractive(); }
void ActionServoTest::perform() { _robot->mrm_servo->test(); }
void ActionStop::perform() { _robot->stopAll(); }
void ActionThermoTest::perform() { _robot->thermoTest(); }
void ActionUS_BTest::perform(){_robot->mrm_us_b->test();}
void ActionUS1Test::perform(){_robot->mrm_us1->test();}
