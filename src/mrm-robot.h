#pragma once
#include "mrm-common.h"
#include <Preferences.h>
#include <mrm-action.h>
#include <mrm-can-bus.h>
#include <mrm-col-b.h>

#define EEPROM_SIZE 12 // EEPROM size
#define LED_ERROR 15 // mrm-esp32's pin number, hardware defined.
#define LED_OK 2 // mrm-esp32's pin number, hardware defined.

#define TEST 0

// Forward declarations

class ActionBase;
class Board;
struct Device;
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
class Mrm_lid_d;
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
private:
	enum ErrorCodes {};
protected:
	static std::map<std::string, ActionBase*>* actions;
	ActionBase* _actionCurrent = NULL; // Current action
	ActionBase* _actionPrevious = NULL; // Previous action

	bool _actionTextDisplay = true;

	uint8_t _boardNextFree = 0;

	uint8_t _devicesAtStartup = 0xFF; // 0xFF - no scan yet
	bool _devicesScanBeforeMenuAndSwitches = true;
	bool _devicesScanOnStartup = true;
	bool devicesScanMenu = true;

	// FPS - frames per second calculation
	uint32_t fpsMs[2] = { 0, 0 };
	uint8_t fpsNextIndex = 0;
	uint32_t fpsTopGap = 0;

	uint8_t menuLevel = 1; // Submenus have bigger numbers
	CANMessage* _msg;
	char _name[16];
	Preferences* preferences; // EEPROM
	bool _sniff = false;
	char _ssid[16];
	char uartRxCommandCumulative[24];
	uint8_t uartRxCommandIndex = 0;
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

	void actionSet(std::string newAction);//ActionBase* newAction);

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

	/** Display number of CAN Bus devices using 8x8 display
	*/
	void devicesLEDCount();

	/** Avoids FPS measuring in the next 2 cycles.
	*/
	void fpsPause();

	/** Updates data for FPS calculation
	*/
	void fpsUpdate();

	/** Resets FPS data
	*/
	void fpsReset();

	/** Enable or disable plug and play for all the connected boards.
	 @param enable - enable or disable
	*/
	void pnpSet(bool enable);

	/** Prints additional data in every loop pass
	*/
	void verbosePrint();

public: 
	std::vector<Board*> boards; // Collection of all the robot's boards

	Errors* errors;
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
	Mrm_lid_d* mrm_lid_d;
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
	void actionAdd(std::string, ActionBase* action);

	/** End current action
	*/
	void actionEnd();

	ActionBase* actionFind(std::string action);

	/** Is this current action's initialization
	@param andFinish - finish initialization
	@return - it is.
	*/
	bool actionPreprocessing(bool andFinish = true);

	/** Finish action's intialization phase
	*/
	void actionPreprocessingEnd();

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

	/** Detects if there is a gap in CAN Bus addresses' sequence of any device, like 0, 2, 3 (missing 1).
	@return - is there a gap.
	*/
	bool canGap();

	void canTest();

	bool canTestParam(int iterations, bool verbose);

	/** Change device's id
	*/
	void canIdChange();

	void canScanToggle();

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

	void colorTest10();

	void colorTest6();

	void colorTest6HSV();

	void colorTestHSV();

	/** The right way to use Arduino function delay
	@param pauseMs - pause in ms. One run even if pauseMs == 0, so that delayMs(0) receives all messages.
	*/
	void delayMs(uint16_t pauseMs);

	/** The right way to use Arduino function delayMicros
	@param pauseMicros - pause in micros. One run even if pauseMicros == 0, so that delayMicross(0) receives all messages.
	*/
	void delayMicros(uint16_t pauseMicros);

	Board* deviceFind(uint16_t msgId, uint8_t& deviceNumber);

	/** Lists all the alive (responded to last ping) CAN Bus devices.
	@boardType - sensor, motor, or all boards
	@return count
	*/
	void deviceInfo(uint8_t deviceOrdinadeviceGlobalOrdinalNumberlNumber, Board * boardFound, Device * deviceFound, Board::BoardType boardType = Board::BoardType::ANY_BOARD);

	void deviceScan();

	/** Contacts all the CAN Bus devices and checks which one is alive.
	@verbose - if true, print.
	@boardType - sensor, motor, or all boards.
	@return count
	*/
	uint8_t devicesScan(bool verbose, Board::BoardType boardType = Board::ANY_BOARD);

	void devicesScan();

	/** Starts devices' CAN Bus messages broadcasting.
	*/
	void devicesStart(uint8_t measuringMode = 0);

	/** Stops broadcasting of CAN Bus messages
	*/
	void devicesStop();

	/** End current action
	*/
	void end() { actionEnd(); }

	void errorsDelete();

	/** Displays errors and stops motors, if any.
	*/
	void errorsDisplay();

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

	/**Compass
	@return - North is 0 degrees, clockwise are positive angles, values 0 - 360.
	*/
	float heading();

	/** Lists I2C devices
	*/
	void i2cTest();

	void imuTest();

	/** Request information
	*/
	void info();

	void irFinderTest();

	void irFinderTestCalculated();

	void led8x8Test();

	/** Tests mrm-lid-can-b
	*/
	void lidar2mTest();

	/** Tests mrm-lid-can-b2
	*/
	void lidar4mTest();

	/** Tests mrm-lid-can-d
	*/
	void lidar4mMultiTest();

	/** Calibrates lidars
	*/
	void lidarCalibrate();

    /** User test, defined in derived classes.
	*/
	virtual void loop() = 0;
	virtual void loop0() = 0;
	virtual void loop1() = 0;
	virtual void loop2() = 0;
	virtual void loop3() = 0;
	virtual void loop4() = 0;

	/** Displays menu
	*/
	void menu();

	/** Color menu
	*/
	void menuColor();

	/** Generic menu
	*/
	void menuLoop();

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

	void messagePrint(CANMessage * message, Board* board = NULL, uint8_t boardIndex = 0xFF, bool outbound = true, bool clientInitiated = false, std::string postfix = "") ;

	/** Receives CAN Bus messages.
	*/
	void messagesReceive(CANMessage message[5], int8_t& last);

	bool messageSend(CANMessage& message); 

	/** Tests motors
	*/
	void motorTest();

	void nodeServoTest();

	/** Tests mrm-node
	*/
	void nodeTest();

	/** Any for or while loop must include call to this function.
	*/
	void noLoopWithoutThis();

	/** Production test
	*/
	void oscillatorTest();

	/** Enable plug and play for all the connected boards.
	 */
	void pnpOn();

	/** Disable plug and play for all the connected boards.
	 */
	void pnpOff();

	/**Pitch
	@return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0 degrees.
	*/
	float pitch();

	void reflectanceArrayCalibrate();

	/** Prints mrm-ref-can* calibration data
	*/
	void reflectanceArrayCalibrationPrint();

	/** Tests mrm-ref-can*
	@digital - digital data. Otherwise analog.
	*/
	void reflectanceArrayTestAnalog();

	void reflectanceArrayTestDigital();

	/** One pass of robot's program
	*/
	void refresh();

	/** Roll
	@return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0 degrees.
	*/
	float roll();

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

	/**
	 * @brief Number of characters in buffer
	 * 
	 * @return Number
	 */
	uint8_t serialDataCount(){return uartRxCommandIndex;}

	/**
	 * @brief Clear buffer
	 */
	void serialDataClear(){uartRxCommandIndex = 0;}

	/**
	 * @brief Returns serial buffer
	 * 
	 * @return buffer
	 */
	char* serialDataGet(){return uartRxCommandCumulative;}

	/** Moves servo motor manually
	*/
	void servoInteractive();

	void servoTest();

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
	void stressTest();

	/** Tests mrm-therm-b-can
	*/
	void thermoTest();

	/** Checks if user tries to break the program
	@return - true if break requested.
	*/
	bool userBreak();

	void us1Test();

	void usBTest();

	/** Verbose output toggle
	*/
	void verboseToggle();
	
#if RADIO == 2
	/** Web server
	*/
	void web();
#endif
};