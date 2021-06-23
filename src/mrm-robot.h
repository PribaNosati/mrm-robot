#pragma once
#include <mrm-action.h>
#include <mrm-can-bus.h>
#include <mrm-col-b.h>
#if RADIO == 1
#include <BluetoothSerial.h>
#endif

#define ACTIONS_LIMIT 80 // Increase if more actions are needed.
#define BOARDS_LIMIT 25 // Maximum number of different board types.
#define LED_ERROR 15 // mrm-esp32's pin number, hardware defined.
#define LED_OK 2 // mrm-esp32's pin number, hardware defined.

// Forward declarations

class Mrm_8x8a;
class Mrm_bldc2x50;
class Mrm_bldc4x2_5;
class Mrm_can_bus;
class Mrm_col_can;
class Mrm_fet_can;
class Mrm_imu;
// class Mrm_ir_finder2;
// class Mrm_ir_finder_can;
class Mrm_ir_finder3;
class Mrm_lid_can_b;
class Mrm_lid_can_b2;
class Mrm_mot2x50;
class Mrm_mot4x10;
class Mrm_mot4x3_6can;
class Mrm_node;
class Mrm_ref_can;
class Mrm_servo;
class Mrm_switch;
class Mrm_therm_b_can;
//class Mrm_us;
class Mrm_us_b;
class Mrm_us1;
#if RADIO == 2
#include "WiFi.h"
#endif

/** Base class for all robots.
*/
class Robot {

protected:
	ActionBase* _action[ACTIONS_LIMIT]; // Collection of all the robot's actions
	uint8_t _actionNextFree = 0;

	// Robot's actions that can be callect directly, not just by iterating _action collection
	ActionBase* _actionCANBusStress;
	ActionBase* _actionCurrent;
	ActionBase* _actionDoNothing;
	ActionBase* _actionLoop;
	ActionBase* _actionMenuMain;
	ActionBase* _actionPrevious;
	ActionBase* _actionStop;

	Board* board[BOARDS_LIMIT]; // Collection of all the robot's boards
	BoardInfo * boardInfo;
	uint8_t _boardNextFree = 0;

	uint8_t _devicesAtStartup = 0;
	bool _devicesScanBeforeMenu = true;

	// FPS - frames per second calculation
	uint32_t fpsMs[2] = { 0, 0 };
	uint8_t fpsNextIndex = 0;
	uint32_t fpsTopGap = 0;

	uint8_t menuLevel = 1; // Submenus have bigger numbers
	CANBusMessage* _msg;
	char _name[16];
	int16_t pitch;
	int16_t roll;
	#if RADIO == 1
	BluetoothSerial *serialBT = NULL;
	#endif
	bool _sniff = false;
	char _ssid[16];
	bool verbose = false; // Verbose output
#if RADIO == 2
	WiFiServer* webServer;
#endif
	char _wiFiPassword[16];

	/** Actually perform the action
	*/
	void actionProcess();

	/** User sets a new action, using keyboard or Bluetooth
	*/
	void actionSet();

	/** New action is set in the program
	@param newAction - the new action.
	*/
	void actionSet(ActionBase* newAction);

	/** Displays all boards
	@return - last board and device's index, 0 if none
	*/
	uint8_t boardsDisplayAll();

	/** Finds board and device's index. Similar to next function, but display choices, too.
	@param selectedBoardIndex - output
	@param selectedDeviceIndex - otuput
	@param maxInput - output
	@param lastBoardAndIndex - output
	@return - true if found
	*/
	bool boardDisplayAndSelect(uint8_t* selectedBoardIndex, uint8_t* selectedDeviceIndex, uint8_t* maxInput, uint8_t* lastBoardAndIndex);

	/** Finds board and device's index for a number received from boardsDisplayAll(). Similar to previous function, but no display.
	@param selectedNumber - input
	@param selectedBoardIndex - output, NULL if none found
	@param selectedDeviceIndex - otuput
	@param maxInput - output
	@return - true if found
	*/
	bool boardSelect(uint8_t selectedNumber, uint8_t *selectedBoardIndex, uint8_t* selectedDeviceIndex, uint8_t* maxInput);

	/** Avoids FPS measuring in the next 2 cycles.
	*/
	void fpsPause();

	/** Updates data for FPS calculation
	*/
	void fpsUpdate();

	/** Resets FPS data
	*/
	void fpsReset();

	/** Prints additional data in every loop pass
	*/
	void verbosePrint();

public: 

	Mrm_can_bus* mrm_can_bus; // CANBus interface
	Mrm_8x8a* mrm_8x8a;
	Mrm_bldc2x50* mrm_bldc2x50;
	Mrm_bldc4x2_5* mrm_bldc4x2_5;
	Mrm_col_b* mrm_col_b;
	Mrm_col_can* mrm_col_can;
	Mrm_fet_can* mrm_fet_can;
	Mrm_imu* mrm_imu;
	// Mrm_ir_finder2* mrm_ir_finder2;
	Mrm_ir_finder3* mrm_ir_finder3;
	// Mrm_ir_finder_can* mrm_ir_finder_can;
	Mrm_lid_can_b* mrm_lid_can_b;// 10
	Mrm_lid_can_b2* mrm_lid_can_b2;
	Mrm_mot2x50* mrm_mot2x50;
	Mrm_mot4x3_6can* mrm_mot4x3_6can;
	Mrm_mot4x10* mrm_mot4x10;
	Mrm_node* mrm_node;
	Mrm_ref_can* mrm_ref_can;
	Mrm_servo* mrm_servo;
	Mrm_switch* mrm_switch;
	Mrm_therm_b_can* mrm_therm_b_can;
	// Mrm_us* mrm_us;
	Mrm_us_b* mrm_us_b;
	Mrm_us1* mrm_us1;

	/**
	*/
	Robot(char name[15] = (char*)"MRMS robot", char ssid[15] = (char*)"MRMS", char wiFiPassword[15] = (char*)"mrms");

	/** Add a new action to the collection of robot's possible actions.
	@param action - the new action.
	*/
	void actionAdd(ActionBase* action);

	/** End current action
	*/
	void actionEnd() { _actionCurrent = NULL; }

	/** Is this current action's initialization
	@param andFinish - finish initialization
	@return - it is.
	*/
	bool actionPreprocessing(bool andFinish = true);

	/** Finish action's intialization phase
	*/
	void actionPreprocessingEnd() { _actionCurrent->preprocessingEnd(); }

	/** Add a new board to the collection of possible boards for the robot
	@param aBoard - the board.
	*/
	void add(Board* aBoard);

	/** Store bitmaps in mrm-led8x8a.
	*/
	virtual void bitmapsSet() = 0;

	/** Blink LED
	*/
	void blink();

	/** Test Bluetooth
	*/
	void bluetoothTest();

	/** Display all the incomming and outcomming CAN Bus messages
	*/
	void canBusSniffToggle();

	/** Change device's id
	*/
	void canIdChange();

	/** mrm-color-can illumination off
	*/
	void colorIlluminationOff();

	/** mrm-color-can illumination on
	*/
	void colorIlluminationOn();

	/** Erase HSV patterns
	*/
	void colorPatternErase();

	/** Print HSV patterns
	*/
	void colorPatternPrint();

	/** Recognize HSV color pattern
	*/
	void colorPatternRecognize();

	/** Record HSV color patterns
	*/
	void colorPatternRecord();

	/** The right way to use Arduino function delay
	@param pauseMs - pause in ms. One run even if pauseMs == 0, so that delayMs(0) receives all messages.
	*/
	void delayMs(uint16_t pauseMs);

	/** The right way to use Arduino function delayMicros
	@param pauseMicros - pause in micros. One run even if pauseMicros == 0, so that delayMicross(0) receives all messages.
	*/
	void delayMicros(uint16_t pauseMicros);

	/** Lists all the alive (responded to last ping) CAN Bus devices.
	@boardType - sensor, motor, or all boards
	@return count
	*/
	void deviceInfo(uint8_t deviceOrdinadeviceGlobalOrdinalNumberlNumber, BoardInfo * deviceInfo, BoardType boardType = ANY_BOARD);

	/** Contacts all the CAN Bus devices and checks which one is alive.
	@verbose - if true, print.
	@boardType - sensor, motor, or all boards
	@return count
	*/
	uint8_t devicesScan(bool verbose, BoardType boardType = ANY_BOARD);

	/** Starts devices' CAN Bus messages broadcasting.
	*/
	void devicesStart(uint8_t measuringMode = 0);

	/** Stops broadcasting of CAN Bus messages
	*/
	void devicesStop();

	/** End current action
	*/
	void end() { actionEnd(); }

	/** Displays errors and stops motors, if any.
	*/
	void errors();

	/** Displays each CAN Bus device's firmware
	*/
	void firmwarePrint();

	/** Returns FPS (frames per second).
	@return - FPS
	*/
	float fpsGet();

	/** Prints FPS all CAN Bus devices and mrm-eps32 boards. Also prints CAN Bus frequency.
	*/
	void fpsPrint();

	/** Orders the robot to go ahead
	*/
	virtual void goAhead() = 0;

	/** Lists I2C devices
	*/
	void i2cTest();

	/** Request information
	*/
	void info();

	/** Tests mrm-lid-can-b
	*/
	void lidar2mTest();

	/** Tests mrm-lid-can-b2
	*/
	void lidar4mTest();

	/** Calibrates lidars
	*/
	void lidarCalibrate();

	/** User test, defined in derived classes.
	*/
	virtual void loop() = 0;

	/** Displays menu
	*/
	void menu();

	/** Color menu
	*/
	void menuColor();

	/** Displays menu and stops motors
	*/
	void menuMainAndIdle();

	/** Reflectance menu
	*/
	void menuReflectance();

	/** System menu
	*/
	void menuSystem();

	/** Print CAN Bus message
	@param msg - message
	@param oubound - if not, inbound
	*/
	void messagePrint(CANBusMessage* msg, bool outbound);

	/** Receives CAN Bus messages.
	*/
	void messagesReceive();

	/** Tests motors
	*/
	void motorTest();

	/** Tests mrm-node
	*/
	void nodeTest();

	/** Any for or while loop must include call to this function.
	*/
	void noLoopWithoutThis();

	/** Production test
	*/
	void oscillatorTest();

	/** Print to all serial ports
	@param fmt - C format string: 
		%c - character,
		%i - integer,
		%s - string.
	@param ... - variable arguments
	*/
	void print(const char* fmt, ...);

	/** Prints mrm-ref-can* calibration data
	*/
	void reflectanceArrayCalibrationPrint();

	/** One pass of robot's program
	*/
	void refresh();

	/** Starts robot's program
	*/
	void run();

	/** Reads serial ASCII input and converts it into an integer
	@param timeoutFirst - timeout for first input
	@param timeoutBetween - timeout between inputs
	@param onlySingleDigitInput - completes input after first digit
	@param limit - returns 0xFFFF if overstepped
	@param printWarnings - prints out of range or timeout warnings
	@return - converted number or 0xFFFF when timeout
	*/
	uint16_t serialReadNumber(uint16_t timeoutFirst = 3000, uint16_t timeoutBetween = 500, bool onlySingleDigitInput = false, uint16_t limit = 0xFFFE, bool printWarnings = true);

	/** Moves servo motor manually
	*/
	void servoInteractive();

	/** Shorthand for actionPreprocessing(). Checks if this is first run.
	@param andFinish - finish initialization
	@return - first run or not.
	*/
	bool setup(bool andFinish = true) {
		return actionPreprocessing(andFinish);
	}

	/** Checks if sniffing is active
	@return - active or not
	*/
	bool sniffing() { return _sniff; }

	/** Stops all motors
	*/
	void stopAll();

	/** CAN Bus stress test
	*/
	bool stressTest();

	/** Tests mrm-therm-b-can
	*/
	void thermoTest();

	/** Checks if user tries to break the program
	@return - true if break requested.
	*/
	bool userBreak();

	/** Verbose output toggle
	*/
	void verboseToggle();
	
	/** Print to all serial ports, pointer to list
	*/
	void vprint(const char* fmt, va_list argp);

#if RADIO == 2
	/** Web server
	*/
	void web();
#endif
};