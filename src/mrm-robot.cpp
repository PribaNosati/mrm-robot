#include <mrm-board.h>
#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-can-bus.h>
#include <mrm-col-b.h>
#include <mrm-col-can.h>
#include <mrm-fet-can.h>
#include <mrm-imu.h>
#include <mrm-pid.h>
//#include <mrm-ir-finder2.h>
//#include <mrm-ir-finder-can.h>
#include <mrm-ir-finder3.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-lid-d.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-node.h>
#include <mrm-ref-can.h>
#include <mrm-robot.h>
#include <mrm-servo.h>
#include <mrm-switch.h>
#include <mrm-therm-b-can.h>
// #include <mrm-us.h>
#include <mrm-us-b.h>
#include <mrm-us1.h>

#if RADIO == 1
extern BluetoothSerial* serialBT;
#endif

/** Print to all serial ports
@param fmt - C format string: 
	%c - character,
	%i - integer,
	%s - string.
@param ... - variable arguments
*/
void Robot::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}


/** Print to all serial ports, pointer to list
*/
void Robot::vprint(const char* fmt, va_list argp) {
	if (strlen(fmt) >= 100)
		return;
	static char buffer[100];
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
#if RADIO == 1
	if (serialBT != NULL)
		serialBT->print(buffer);
#endif
}

/**
*/
Robot::Robot(char name[15], char ssid[15], char wiFiPassword[15]) {
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
	Serial.begin(115200);

	if (strlen(name) > 15)
		strcpy(errorMessage, "Name overflow");
	strcpy(_name, name);

	if (strlen(ssid) > 15)
		strcpy(errorMessage, "SSID overflow");
	strcpy(_ssid, ssid);

	if (strlen(wiFiPassword) > 15)
		strcpy(errorMessage, "WiFi pwd. overflow");
	strcpy(_wiFiPassword, wiFiPassword);
	boardInfo = new BoardInfo();

	// EEPROM, data retained after system powered down
	preferences = new Preferences();
	preferences->begin("data", false);

#if RADIO == 1
	if (serialBT == NULL) {
		serialBT = new BluetoothSerial(); // Additional serial port
		serialBT->begin(_name); //Start Bluetooth. ESP32 - Bluetooth device name, choose one.
	}
#endif

	delay(50);
	print("%s started.\r\n", _name);

	Wire.begin(); // Start I2C

#if RADIO == 2
	delay(100);
	webServer = new WiFiServer(80);

	  // Connect to Wi-Fi network with SSID and password
	print("Connecting to %s", ssid);
	WiFi.begin(ssid, wiFiPassword);
	uint32_t startMs = millis();
	bool ok = true;
	while (WiFi.status() != WL_CONNECTED) {
		delay(200);
		print(".");
		if (millis() - startMs > 2000){
			ok = false;
			break;
		}
	}
	if (ok){
		// Print local IP address and start web server
		print("\n\r");
		print("WiFi connected.\n\r");
		print("IP address: %i.%i.%i.%i\n\r", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
		webServer->begin();
	}
	else
		print("\n\rWeb server not started.\n\r");
#endif

	mrm_can_bus = new Mrm_can_bus();

	// LED Test
	LEDSignText* signTest = new LEDSignText();
	strcpy(signTest->text, "Test");

	_actionCurrent = NULL;
	_actionPrevious = _actionCurrent;

	_actionCANBusStress = new ActionCANBusStress(this);
	_actionDoNothing = new ActionDoNothing(this);
	_actionLoop = new ActionLoop(this, signTest);
	_actionMenuMain = new ActionMenuMain(this);
	_actionStop = new ActionStop(this);

	actionAdd(new Action8x8Test(this));
	actionAdd(new ActionBluetoothTest(this, signTest));
	actionAdd(new ActionCANBusScan(this));
	actionAdd(new ActionCANBusSniff(this));
	actionAdd(new ActionCANBusStress(this));
	actionAdd(new ActionColorBTest6Colors(this, signTest));
	actionAdd(new ActionColorBTestHSV(this, signTest));
	actionAdd(new ActionColorIlluminationOff(this));
	actionAdd(new ActionColorIlluminationOn(this));
	actionAdd(new ActionColorPatternErase(this));
	actionAdd(new ActionColorPatternPrint(this));
	actionAdd(new ActionColorPatternRecognize(this));
	actionAdd(new ActionColorPatternRecord(this));
	actionAdd(new ActionColorTest6Colors(this, signTest));
	actionAdd(new ActionColorTestHSV(this, signTest));
	actionAdd(new ActionDeviceIdChange(this));
	actionAdd(new ActionFirmware(this));
	actionAdd(new ActionFPS(this));
	actionAdd(new ActionGoAhead(this));
	actionAdd(new ActionI2CTest(this, signTest));
	actionAdd(new ActionIMUTest(this, signTest));
	actionAdd(new ActionInfo(this));
	actionAdd(new ActionIRFinderTest(this, signTest));
	actionAdd(new ActionIRFinderCanTest(this, signTest));
	actionAdd(new ActionIRFinderCanTestCalculated(this, signTest));
	actionAdd(new ActionLidar2mTest(this, signTest));
	actionAdd(new ActionLidar4mTest(this, signTest));
	actionAdd(new ActionLidar4mMultiTest(this, signTest));
	actionAdd(new ActionLidarCalibrate(this));
	actionAdd(_actionLoop);
	actionAdd(new ActionMenuColor(this));
	actionAdd(new ActionMenuColorB(this));
	actionAdd(new ActionMenuMain(this));
	actionAdd(new ActionMenuReflectance(this));
	actionAdd(new ActionMenuSystem(this));
	actionAdd(new ActionMotorTest(this, signTest));
	actionAdd(new ActionNodeTest(this, signTest));
	actionAdd(new ActionNodeServoTest(this, signTest));
	//actionAdd(new ActionOscillatorTest(this));
	actionAdd(new ActionPnPOff(this));
	actionAdd(new ActionPnPOn(this));
	actionAdd(new ActionReflectanceArrayCalibrate(this));
	actionAdd(new ActionReflectanceArrayCalibrationPrint(this));
	actionAdd(new ActionReflectanceArrayAnalogTest(this, signTest));
	actionAdd(new ActionReflectanceArrayDigitalTest(this, signTest));
	actionAdd(new ActionServoInteractive(this));
	actionAdd(new ActionServoTest(this, signTest));
	actionAdd(_actionStop);
	actionAdd(new ActionThermoTest(this, signTest));
	actionAdd(new ActionUS_BTest(this, signTest));
	actionAdd(new ActionUS1Test(this, signTest));

	mrm_8x8a = new Mrm_8x8a(this);
	mrm_bldc2x50 = new Mrm_bldc2x50(this);
	mrm_bldc4x2_5 = new Mrm_bldc4x2_5(this);
	mrm_col_b = new Mrm_col_b(this);
	mrm_col_can = new Mrm_col_can(this);
	mrm_fet_can = new Mrm_fet_can(this);
	mrm_imu = new Mrm_imu(this);
	//mrm_ir_finder2 = new Mrm_ir_finder2(this);
	// mrm_ir_finder_can = new Mrm_ir_finder_can(this);
	mrm_ir_finder3 = new Mrm_ir_finder3(this);
	mrm_lid_can_b = new Mrm_lid_can_b(this);
	mrm_lid_can_b2 = new Mrm_lid_can_b2(this);
	mrm_lid_d = new Mrm_lid_d(this);
	mrm_mot2x50 = new Mrm_mot2x50(this);
	mrm_mot4x3_6can = new Mrm_mot4x3_6can(this);
	mrm_mot4x10 = new Mrm_mot4x10(this);
	mrm_node = new Mrm_node(this);
	mrm_ref_can = new Mrm_ref_can(this);
	mrm_servo = new Mrm_servo(this);
	mrm_switch = new Mrm_switch(this);
	mrm_therm_b_can = new Mrm_therm_b_can(this);
	// mrm_us = new Mrm_us(this);
	mrm_us_b = new Mrm_us_b(this);
	mrm_us1 = new Mrm_us1(this);

	// 8x8 LED
	mrm_8x8a->add((char*)"LED8x8-0");

	// Motors mrm-bldc2x50
	mrm_bldc2x50->add(false, (char*)"BL2x50-0");
	mrm_bldc2x50->add(false, (char*)"BL2x50-1");
	mrm_bldc2x50->add(false, (char*)"BL2x50-2");
	mrm_bldc2x50->add(false, (char*)"BL2x50-3");

	// LEDs
	pinMode(2, OUTPUT);
	digitalWrite(2, false);
	pinMode(LED_ERROR, OUTPUT); // Error LED
	digitalWrite(15, false);

	// Motors mrm-bldc4x2.5
	mrm_bldc4x2_5->add(false, (char*)"BL4x2.5-0");
	mrm_bldc4x2_5->add(false, (char*)"BL4x2.5-1");
	mrm_bldc4x2_5->add(false, (char*)"BL4x2.5-2");
	mrm_bldc4x2_5->add(false, (char*)"BL4x2.5-3");

	// Colors sensors mrm-col-b
	mrm_col_b->add((char*)"Clr-0");
	mrm_col_b->add((char*)"Clr-1");

	// Colors sensors mrm-col-can
	mrm_col_can->add((char*)"Col-0");
	mrm_col_can->add((char*)"Col-1");
	mrm_col_can->add((char*)"Col-2");
	mrm_col_can->add((char*)"Col-3");

	// FET outputs
	mrm_fet_can->add((char*)"FET-0");

	// IMU
	mrm_imu->add();

	// // mrm-ir-finder2
	// mrm_ir_finder3->add(34, 33);

	// // mrm-ir-finder-can
	// mrm_ir_finder_can->add((char*)"IRFind-0");

	// mrm-ir-finder3
	mrm_ir_finder3->add((char*)"IR3Fin-0");

	// Motors mrm-mot2x50
	mrm_mot2x50->add(false, (char*)"Mot2x50-0");
	mrm_mot2x50->add(false, (char*)"Mot2x50-1");
	mrm_mot2x50->add(false, (char*)"Mot2x50-2");
	mrm_mot2x50->add(false, (char*)"Mot2x50-3");
	mrm_mot2x50->add(false, (char*)"Mot2x50-4");
	mrm_mot2x50->add(false, (char*)"Mot2x50-5");

	// Motors mrm-mot4x10
	mrm_mot4x10->add(false, (char*)"Mot4x10-0");
	mrm_mot4x10->add(false, (char*)"Mot4x10-1");
	mrm_mot4x10->add(false, (char*)"Mot4x10-2");
	mrm_mot4x10->add(false, (char*)"Mot4x10-3");

	// Motors mrm-mot4x3.6can
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-0");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-1");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-2");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-3");

	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-4");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-5");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-6");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-7");

	// Lidars mrm-lid-can-b, VL53L0X, 2 m
	mrm_lid_can_b->add((char*)"Lidar2m-0");
	mrm_lid_can_b->add((char*)"Lidar2m-1");
	mrm_lid_can_b->add((char*)"Lidar2m-2");
	mrm_lid_can_b->add((char*)"Lidar2m-3");
	mrm_lid_can_b->add((char*)"Lidar2m-4");
	mrm_lid_can_b->add((char*)"Lidar2m-5");
	mrm_lid_can_b->add((char*)"Lidar2m-6");
	mrm_lid_can_b->add((char*)"Lidar2m-7");
	mrm_lid_can_b->add((char*)"Lidar2m-8");
	mrm_lid_can_b->add((char*)"Lidar2m-9");
	mrm_lid_can_b->add((char*)"Lidar2m10");
	mrm_lid_can_b->add((char*)"Lidar2m11");
	mrm_lid_can_b->add((char*)"Lidar2m12");
	mrm_lid_can_b->add((char*)"Lidar2m13");

	// Lidars mrm-lid-can-b2, VL53L1X, 4 m
	mrm_lid_can_b2->add((char*)"Lidar4m-0");
	mrm_lid_can_b2->add((char*)"Lidar4m-1");
	mrm_lid_can_b2->add((char*)"Lidar4m-2");
	mrm_lid_can_b2->add((char*)"Lidar4m-3");
	mrm_lid_can_b2->add((char*)"Lidar4m-4");
	mrm_lid_can_b2->add((char*)"Lidar4m-5");
	mrm_lid_can_b2->add((char*)"Lidar4m-6");
	mrm_lid_can_b2->add((char*)"Lidar4m-7");

	// Lidars mrm-lid-can-b2, VL53L5X, 4 m multi
	mrm_lid_d->add((char*)"LidMul-0");
	// mrm_lid_d->add((char*)"LidMul-1");

	// CAN Bus node
	mrm_node->add((char*)"Node-0");
	mrm_node->add((char*)"Node-1");

	// Reflective array
	mrm_ref_can->add((char*)"RefArr-0");
	mrm_ref_can->add((char*)"RefArr-1");
	mrm_ref_can->add((char*)"RefArr-2");
	mrm_ref_can->add((char*)"RefArr-3");
	mrm_ref_can->add((char*)"RefArr-4");

	// Switch
	mrm_switch->add(18, 19, (char*)"Switch");

	// Thermal array
	mrm_therm_b_can->add((char*)"Thermo-0");
	mrm_therm_b_can->add((char*)"Thermo-1");
	mrm_therm_b_can->add((char*)"Thermo-2");
	mrm_therm_b_can->add((char*)"Thermo-3");

	// Ultrasonic
	mrm_us_b->add((char*)"US-B-0");
	mrm_us_b->add((char*)"US-B-1");
	mrm_us_b->add((char*)"US-B-2");
	mrm_us_b->add((char*)"US-B-3");
	mrm_us_b->add((char*)"US-B-4");

	mrm_us1->add((char*)"US1-0");
	// mrm_us->add((char*)"US-0");
	// mrm_us->add((char*)"US-1");
	// mrm_us->add((char*)"US-2");
	// mrm_us->add((char*)"US-3");

	// Add boards
	add(mrm_8x8a);
	add(mrm_bldc2x50);
	add(mrm_bldc4x2_5);
	add(mrm_col_b);
	add(mrm_col_can);
	add(mrm_fet_can);
	add(mrm_ir_finder3);
	add(mrm_lid_can_b);
	add(mrm_lid_can_b2);
	add(mrm_lid_d);
	add(mrm_mot2x50);
	add(mrm_mot4x10);
	add(mrm_mot4x3_6can);
	add(mrm_node);
	add(mrm_ref_can);
	add(mrm_therm_b_can);
	// add(mrm_us);
	add(mrm_us_b);
	add(mrm_us1);

	_devicesAtStartup = devicesScan(true);
	devicesLEDCount();
}

/** Add a new action to the collection of robot's possible actions.
@param action - the new action.
*/
void Robot::actionAdd(ActionBase* action) {
	if (_actionNextFree >= ACTIONS_LIMIT) {
		strcpy(errorMessage, "ACTIONS_LIMIT exceeded.");
		return;
	}
	_action[_actionNextFree++] = action;
}

/** Is this current action's initialization
@param andFinish - finish initialization
@return - it is.
*/
bool Robot::actionPreprocessing(bool andFinish) {
	bool itIs = _actionCurrent->preprocessing();
	if (andFinish)
		_actionCurrent->preprocessingEnd();
	return itIs;
}

/** Actually perform the action
*/
void Robot::actionProcess() {
	if (_actionCurrent != NULL) {
		if (_actionCurrent->preprocessing()) {
			_actionCurrent->performBefore();
		}
		_actionCurrent->perform();
		//if (_actionCurrent != NULL)
		//	_actionCurrent->_preprocessing = false;
	}
}

/** User sets a new action, using keyboard or Bluetooth
*/
void Robot::actionSet() {
	static uint32_t lastUserActionMs = 0;
	const uint16_t TIMEOUT_MS = 2000;

	// If a button pressed, first execute its action
	ActionBase* action8x8 = NULL;
	if (mrm_8x8a->alive()) 
		action8x8 = mrm_8x8a->actionCheck(); 
	ActionBase* actionSw = mrm_switch->actionCheck(); 
	if (action8x8 != NULL)
		actionSet(action8x8);
	else if (actionSw != NULL)
		actionSet(actionSw);
	else { // Check keyboard
		bool btAvailable = false;
#if RADIO == 1
		btAvailable = serialBT != NULL && serialBT->available();
#endif
		if (Serial.available() || btAvailable) {
			lastUserActionMs = millis();
			uint8_t ch = ' ';
			if (Serial.available())
				ch = Serial.read();
#if RADIO == 1
			else
				if (serialBT != NULL)
					ch = serialBT->read();
#endif

			if (ch != 13) //if received data different from ascii 13 (enter)
				uartRxCommandCumulative[uartRxCommandIndex++] = ch;	//add data to Rx_Buffer

			if (ch == 13 || (uartRxCommandIndex >= 3 && !(uartRxCommandCumulative[0] == 'e' && uartRxCommandCumulative[1] == 's' && uartRxCommandCumulative[2] == 'c')) || ch == 'x' && uartRxCommandIndex == 1) //if received data = 13
			{

				print("Command: %s", uartRxCommandCumulative);

				uint8_t found = 0;
				if (uartRxCommandCumulative[0] == 'e' && uartRxCommandCumulative[1] == 's' && uartRxCommandCumulative[2] == 'c'){
					found = 1;
				}
				else{
					uartRxCommandCumulative[uartRxCommandIndex] = 0;
					uartRxCommandIndex = 0;
				}

				for (uint8_t i = 0; i < _actionNextFree; i++) {
					if (strcmp(_action[i]->_shortcut, uartRxCommandCumulative) == 0) {
						print(" ok.\r\n");
						actionSet(_action[i]);
						//commandPrevious = commandCurrent;
						//commandCurrent = commands[i];
						//commandCurrent->firstProcess = true;
						found = 1;
						break;
					}
				}
				if (!found) {
					print(" not found.\r\n");
					uartRxCommandIndex = 0;
				}
			}
		}

		if (uartRxCommandIndex != 0 && millis() - lastUserActionMs > TIMEOUT_MS) {
			print(" Timeout.\r\n");
			uartRxCommandIndex = 0;
		}
	}
}

/** New action is set in the program
@param newAction - the new action.
*/
void Robot::actionSet(ActionBase* newAction) {
	_actionPrevious = _actionCurrent;
	_actionCurrent = newAction;
	_actionCurrent->preprocessingStart();
	// Display action on 8x8 LED
	if (mrm_8x8a->alive() && _actionTextDisplay){
		if (_actionCurrent->ledSign == NULL)
			devicesLEDCount();
		else if (_actionCurrent->ledSign->type == 1 && strcmp(((LEDSignText*)(_actionCurrent->ledSign))->text, "") != 0)
			mrm_8x8a->text(((LEDSignText*)(_actionCurrent->ledSign))->text);
		else if (_actionCurrent->ledSign->type == 0){
			mrm_8x8a->bitmapCustomDisplay(
				((LEDSignBitmap*)_actionCurrent->ledSign)->red, 
				((LEDSignBitmap*)_actionCurrent->ledSign)->green);
		}
	}
}

/** Add a new board to the collection of possible boards for the robot
@param aBoard - the board.
*/
void Robot::add(Board* aBoard) {
	if (_boardNextFree > BOARDS_LIMIT - 1) {
		strcpy(errorMessage, "Too many boards");
		return;
	}
	board[_boardNextFree++] = aBoard;
}

/** Blink LED
*/
void Robot::blink() {
	const uint16_t onMs = 100;
	const uint16_t offMs = 1000;
	uint8_t repeatOnTimes;
	static uint32_t lastBlinkMs = 0;
	static uint8_t isOn = 0;
	static uint8_t pass = 0;

	if (strcmp(errorMessage, "") == 0)
		repeatOnTimes = 1;
	else
		repeatOnTimes = 2;

	if (pass < repeatOnTimes) {
		if (millis() - lastBlinkMs > onMs) {
			isOn = !isOn;
			if (!isOn)
				pass++;
			digitalWrite(LED_OK, isOn);
			lastBlinkMs = millis();
		}
	}
	else if (millis() - lastBlinkMs > offMs) {
		pass = 0;
		lastBlinkMs = 0;
	}
}

/** Displays all boards
@return - last board and device's index, 0 if none
*/
uint8_t Robot::boardsDisplayAll() {
	// Print all devices alive
	uint8_t last = 0;
	for (uint8_t boardNumber = 0; boardNumber < _boardNextFree; boardNumber++) {
		uint8_t currentCount = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount(); deviceNumber++)
			if (board[boardNumber]->alive(deviceNumber)) {
				if (currentCount == 0)
					print("%i.", ++last);
				else
					print(",");
				print(" %s", board[boardNumber]->name(deviceNumber));
				if (++currentCount == board[boardNumber]->devicesOnASingleBoard()) {
					currentCount = 0;
					print("\n\r");
				}
			}
	}
	if (last == 0)
		print("No boards\n\r");
	return last;
}

/** Finds board and device's index. Similar to next function, but display choices, too.
@param selectedBoardIndex - output
@param selectedDeviceIndex - otuput
@param maxInput - output
@param lastBoardAndIndex - output
@return - true if found
*/
bool Robot::boardDisplayAndSelect(uint8_t *selectedBoardIndex, uint8_t* selectedDeviceIndex, uint8_t* maxInput, uint8_t* lastBoardAndIndex) {
	*lastBoardAndIndex = boardsDisplayAll();
	bool found = false;
	if (*lastBoardAndIndex > 0) {

		// Choose device
		print("Enter board [1 - %i]: ", *lastBoardAndIndex);
		uint16_t selectedNumber = serialReadNumber(15000, 500, *lastBoardAndIndex <= 9, *lastBoardAndIndex);
		print("%i", selectedNumber);

		if (selectedNumber != 0xFFFF)
			found = boardSelect(selectedNumber, selectedBoardIndex, selectedDeviceIndex, maxInput);

	}
	return found;
}

/** Finds board and device's index for a number received from boardsDisplayAll()
@param selectedNumber - input
@param selectedBoardIndex - output
@param selectedDeviceIndex - otuput
@param maxInput - output
@return - true if found
*/
bool Robot::boardSelect(uint8_t selectedNumber, uint8_t *selectedBoardIndex, uint8_t * selectedDeviceIndex, uint8_t *maxInput) {
	// Find selected board
	uint8_t lastBoardAndIndex = 0;
	*selectedBoardIndex = 0;
	*selectedDeviceIndex = 0xFF;
	*maxInput = 0;
	for (uint8_t boardNumber = 0; boardNumber < _boardNextFree && *selectedDeviceIndex == 0xFF; boardNumber++) {
		uint8_t currentCount = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount() && *selectedDeviceIndex == 0xFF; deviceNumber++)
			if (board[boardNumber]->alive(deviceNumber)) {
				if (currentCount == 0) {
					if (++lastBoardAndIndex == selectedNumber) {
						// Board *selectedBoard = board[boardNumber];
						*selectedDeviceIndex = deviceNumber;
						*selectedBoardIndex = boardNumber;
						*maxInput = board[boardNumber]->deadOrAliveCount() / board[boardNumber]->devicesOnASingleBoard() - 1;
						break;
					}
				}

				if (++currentCount == board[boardNumber]->devicesOnASingleBoard())
					currentCount = 0;
			}
	}
	return *selectedDeviceIndex != 0xFF;
}

/** Test Bluetooth
*/
void Robot::bluetoothTest() {
	static uint32_t startMs = millis();
	if (millis() - startMs > 100) {
		print("Time: %i ms.\r\n", millis() - startMs);
		startMs = millis();
	}
}

/** Display all the incomming and outcomming CAN Bus messages
*/
void Robot::canBusSniffToggle() {
	_sniff = !_sniff;
	if (_sniff)
		print("Sniff on\n\r");
	else
		print("Sniff off\n\r");
	end();
}

/** Detects if there is a gap in CAN Bus addresses' sequence of any device, like 0, 2, 3 (missing 1).
@return - is there a gap.
*/
bool Robot::canGap() {
	for (uint8_t i = 0; i < _boardNextFree; i++)
		if (board[i]->canGap())
			return true;
	return false;
}

/** Change device's id
*/
void Robot::canIdChange() {
	uint8_t selectedBoardIndex;
	uint8_t selectedDeviceIndex;
	uint8_t maxInput;
	uint8_t lastBoardAndDeviceIndex;
	if (boardDisplayAndSelect(&selectedBoardIndex, &selectedDeviceIndex, &maxInput, &lastBoardAndDeviceIndex)) {
		// Enter new id
		print(". %s\n\rEnter new board id [0..%i]: ", board[selectedBoardIndex]->name(), maxInput);
		uint8_t newDeviceNumber = serialReadNumber(15000, 500, maxInput <= 9, maxInput);

		if (newDeviceNumber != 0xFF) {
			// Change
			print("%i\n\rChange requested.\n\r", newDeviceNumber);
				board[selectedBoardIndex]->idChange(newDeviceNumber, selectedDeviceIndex);
			delayMs(500); // Delay for firmware handling of devices with the same ids.
		}
	}

	end();
}

/** mrm-color-can illumination off
*/
void Robot::colorIlluminationOff() {
	if (mrm_col_can->alive())
		mrm_col_can->illumination(0xFF, 0);
	else if (mrm_col_b->alive())
		mrm_col_b->illumination(0xFF, 0);
	end();
}

/** mrm-color-can illumination on
*/
void Robot::colorIlluminationOn() {
	if (mrm_col_can->alive())
		mrm_col_can->illumination(0xFF, 5);
	else if (mrm_col_b->alive())
		mrm_col_b->illumination(0xFF, 5);
	end();
}

/** Erase HSV patterns
*/
void Robot::colorPatternErase() {
	if (mrm_col_can->alive())
		mrm_col_can->patternErase();
	else if (mrm_col_b->alive())
		mrm_col_b->patternErase();
	end();
}

/** Print HSV patterns
*/
void Robot::colorPatternPrint() {
	if (mrm_col_can->alive())
		mrm_col_can->patternPrint();
	else if (mrm_col_b->alive())
		mrm_col_b->patternPrint();
	end();
}

/** Record HSV color patterns
*/
void Robot::colorPatternRecord() {
	if (mrm_col_can->alive())
		mrm_col_can->patternsRecord();
	else if (mrm_col_b->alive())
		mrm_col_b->patternsRecord();
	end();
}

/** Recognize HSV color pattern
*/
void Robot::colorPatternRecognize() {
	end();
}

/** The right way to use Arduino function delay
@param pauseMs - pause in ms. One run even if pauseMs == 0, so that delayMs(0) receives all messages.
*/
void Robot::delayMs(uint16_t pauseMs) {
	uint32_t startMs = millis();
	do  {
		noLoopWithoutThis();
	} while (millis() - startMs < pauseMs);
}

/** The right way to use Arduino function delayMicros
@param pauseMicros - pause in micros. One run even if pauseMicros == 0, so that delayMicross(0) receives all messages.
*/
void Robot::delayMicros(uint16_t pauseMicros) {
	uint32_t startMicros = micros();
	do {
		noLoopWithoutThis();
		if (startMicros > micros()) // micros() will overflow after 72 min!
			startMicros = 0; // In that case, reset the start time
	} while (micros() < startMicros + pauseMicros);
}

/** Lists all the alive (responded to last ping) CAN Bus devices.
@boardType - sensor, motor, or all boards
@return count
*/
void Robot::deviceInfo(uint8_t deviceGlobalOrdinalNumber, BoardInfo * deviceInfo, BoardType boardType){
	uint8_t count = 0;
	for (uint8_t boardKind = 0; boardKind < _boardNextFree; boardKind++){
		if (boardType == ANY_BOARD || board[boardKind]->boardType() == boardType){ // Board types
			for (uint8_t deviceNumber = 0; deviceNumber < board[boardKind]->count(); deviceNumber++){// Devices for the current board type
				if (board[boardKind]->alive(deviceNumber)){
					if (count == deviceGlobalOrdinalNumber)
					{
						strcpy(deviceInfo->name, board[boardKind]->name(deviceNumber));
						deviceInfo->board = board[boardKind];
						deviceInfo->deviceNumber = deviceNumber;
						//print("In func: %s %i", deviceInfo->name, deviceNumber);
						if (boardType == SENSOR_BOARD)
							deviceInfo->readingsCount = ((SensorBoard*)(board[boardKind]))->readingsCount();
						return;
					}
					else
						count++;
				}
			}
		}
	}
	strcpy(deviceInfo->name, "");
	deviceInfo->readingsCount = 0;
}

/** Display number of CAN Bus devices using 8x8 display
*/
void Robot::devicesLEDCount(){
	if (mrm_8x8a->alive()){
		char buffer[7];
		sprintf(buffer, "N:%i.", _devicesAtStartup);
		mrm_8x8a->text(buffer);
	}
}

/** Contacts all the CAN Bus devices and checks which one is alive.
@verbose - if true, print.
@boardType - sensor, motor, or all boards
@return count
*/
uint8_t Robot::devicesScan(bool verbose, BoardType boardType) {
	devicesStop();
	uint8_t count = 0;
	delayMs(50); // Read all the messages sent after stop.
	for (uint8_t i = 0; i < _boardNextFree; i++){
		if (boardType == ANY_BOARD || board[i]->boardType() == boardType)
			count += board[i]->devicesScan(verbose);
			// print("SC1 %s ", board[i]->name()),count += board[i]->devicesScan(verbose), print("SC2");
	}
	if (verbose)
		print("%i devices.\n\r", count);
	if (canGap())
		strcpy(errorMessage, "CAN gap");
	end();
	return count;
}

/** Starts devices' CAN Bus messages broadcasting.
*/
void Robot::devicesStart(uint8_t measuringMode) {
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++)
		board[deviceNumber]->start(0xFF, measuringMode);
}

/** Stops broadcasting of CAN Bus messages
*/
void Robot::devicesStop() {
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++) {
		board[deviceNumber]->stop();
		delayMs(1); // TODO
	}
}

/** Displays errors and stops motors, if any.
*/
void Robot::errors() {
	if (strcmp(errorMessage, "") != 0) {
		print("Error! %s\n\r", errorMessage);
		strcpy(errorMessage, "");
		stopAll(); // Stop all motors
		end();
	}
}

/** Displays each CAN Bus device's firmware
*/
void Robot::firmwarePrint() {
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->firmwareRequest();
		// uint32_t startMs = millis();
		delayMs(1);
	}
	end();
}

/** Returns FPS (frames per second).
@return - FPS
*/
float Robot::fpsGet() {
	float fpsNow;
	//print("Next: %i %i %i\n\r", fpsNextIndex, fpsMs[0], fpsMs[1]);
	if (fpsMs[1] == 0 || fpsMs[0] == 0)
		fpsNow = 0;
	else if (fpsMs[0] == fpsMs[1])
		fpsNow = 1000000;
	else
		fpsNow = 1000000.0 / (float)(fpsMs[1] > fpsMs[0] ? fpsMs[1] - fpsMs[0] : fpsMs[0] - fpsMs[1]);
	return fpsNow;
}

/** Avoids FPS measuring in the next 2 cycles.
*/
void Robot::fpsPause() {
	fpsMs[0] = 0;
	fpsMs[1] = 0;
}

/** Prints FPS all CAN Bus devices and mrm-eps32 boards. Also prints CAN Bus frequency.
*/
void Robot::fpsPrint() {
	print("CAN peaks: %i received/s, %i sent/s\n\r", mrm_can_bus->messagesPeakReceived(), mrm_can_bus->messagesPeakSent());
	print("Arduino: %i FPS, low peak: %i FPS\n\r", (int)fpsGet(), fpsTopGap == 1000 ? 0 : (int)(1000 / (float)fpsTopGap));
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->fpsRequest();
		uint32_t startMs = millis();
		while (millis() - startMs < 30)
			noLoopWithoutThis();
		board[i]->fpsDisplay();
	}
	fpsReset();
	end();
}

/** Resets FPS data
*/
void Robot::fpsReset() {
	fpsTopGap = 0;
	fpsMs[0] = 0;
	fpsMs[1] = 0;
	mrm_can_bus->messagesReset();
}

/** Updates data for FPS calculation
*/
void Robot::fpsUpdate() {
	fpsMs[fpsNextIndex] = millis(); // millis() will overflow after some time! Not taken into account here.
	fpsNextIndex = (fpsNextIndex == 0 ? 1 : 0);
	if (fpsMs[0] != 0 && fpsMs[1] != 0) {
		uint32_t gap = (fpsNextIndex == 0 ? fpsMs[1] - fpsMs[0] : fpsMs[0] - fpsMs[1]);
		if (gap > fpsTopGap)
			fpsTopGap = gap;
	}
}


/**Compass
@return - North is 0 degrees, clockwise are positive angles, values 0 - 360.
*/
float Robot::heading() {
	return mrm_imu->heading();
}


/** Lists I2C devices
*/
void Robot::i2cTest() {
	print("Scanning.\n\r");

	bool any = false;
	for (byte address = 1; address < 127; address++)
	{
		Wire.beginTransmission(address); // Transmission tried
		byte status = Wire.endTransmission(); // Was it successful?
		if (status == 0)
		{
			print("Found at address 0x%02x\n\r", address);
			any = true;
		}
		else if (status == 4)
			print("Found at address 0x%02x\n\r", address);
	}
	if (!any)
		print("Nothing found.\n\n\r");

	end();
}

/** Request information
*/
void Robot::info() {
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->info();
		delay(1);
	}
	end();
}

/** Tests mrm-lid-can-b
*/
void Robot::lidar2mTest() {
	static uint16_t selected;
	if (setup()) {
		//devicesScan(false, SENSOR_BOARD);
		// Select lidar
		uint8_t count = mrm_lid_can_b->deadOrAliveCount();
		print("%s - enter lidar number [0-%i] or wait for all\n\r", mrm_lid_can_b->name(), count - 1);
		selected = serialReadNumber(2000, 1000, count - 1 < 9, count - 1, false);
		if (selected == 0xFFFF) { // Test all
			print("Test all\n\r");
			selected = 0xFF;
		}
		else { // Test only selected
			if (mrm_lid_can_b->alive(selected))
				print("\n\rTest lidar %s\n\r", mrm_lid_can_b->name(selected));
			else {
				print("\n\rLidar %s dead, test all\n\r", mrm_lid_can_b->name(selected));
				selected = 0xFF;
			}
		}
		//mrm_lid_can_b->start(selected);
	}
	mrm_lid_can_b->test(selected);
}

/** Tests mrm-lid-can-b2
*/
void Robot::lidar4mTest() {
	if (setup())
		mrm_lid_can_b2->start();
	mrm_lid_can_b2->test();
}

/** Calibrates lidars
*/
void Robot::lidarCalibrate() {
	print("Lidar calibration\n\r");

	// Select lidar 2 or 4 m
	int8_t selected2Or4 = -1;
	uint32_t lastMs;
	while (selected2Or4 != 2 && selected2Or4 != 4) {
		print("Enter max distance [2 or 4]m or wait to abort ");
		lastMs = millis();
		selected2Or4 = -1;
		while (millis() - lastMs < 10000 && selected2Or4 == -1)
			if (Serial.available()) {
				uint8_t ch = Serial.read() - 48;
				print("%i\n\r", ch);
				selected2Or4 = ch;
			}
		if (selected2Or4 == -1) {
			print("- abort\n\r");
			break;
		}
	}

	// Select lidar number
	if (selected2Or4 != -1) {
		// Select lidar
		uint8_t count = selected2Or4 == 2 ? mrm_lid_can_b->deadOrAliveCount() : mrm_lid_can_b2->deadOrAliveCount();
		print("Enter lidar number [0-%i] or wait to abort", count - 1);
		uint16_t selected = serialReadNumber(3000, 1000, count - 1 < 9, count - 1, false);
		if (selected == 0xFFFF)
			print("\n\rAbort\n\r");
		else {
			if (selected2Or4 == 2 ? mrm_lid_can_b->alive(selected) : mrm_lid_can_b2->alive(selected)) {
				print("\n\rCalibrate lidar %s\n\r", mrm_lid_can_b->name(selected));
				selected2Or4 == 2 ? mrm_lid_can_b->calibration(selected) : mrm_lid_can_b2->calibration(selected);
			}
			else
				print("\n\rLidar %s dead\n\r", selected2Or4 == 2 ? mrm_lid_can_b->name(selected) : mrm_lid_can_b2->name(selected));
		}
	}

	end();
}

/** Displays menu
*/
void Robot::menu() {
	// Print menu
	if (_devicesScanBeforeMenu){
		uint8_t cnt = devicesScan(false);
		if (cnt > _devicesAtStartup)  // Late-booters
			_devicesAtStartup = cnt;
		else if (cnt < _devicesAtStartup){
			print("%i devices instead of %i!\n\r", cnt, _devicesAtStartup);
			if (mrm_8x8a->alive(0, false))
				mrm_8x8a->text((char*)"Error. Cnt.");
		}
	}
	print("\r\n");

	bool any = false;
	uint8_t column = 1;
	uint8_t maxColumns = 2;
	for (uint8_t i = 0; i < _actionNextFree; i++) {
		if ((_action[i]->_menuLevel | menuLevel) == _action[i]->_menuLevel) {
			bool anyAlive = false;
			if (_action[i]->boardsId() == ID_ANY)
				anyAlive = true;
			else
				for (uint8_t j = 0; j < _boardNextFree && !anyAlive; j++)
					if (board[j]->alive(0xFF) && board[j]->id() == _action[i]->boardsId())
						anyAlive = true;
			if (anyAlive) {
				print("%-3s - %-15s%s", _action[i]->_shortcut, _action[i]->_text, column == maxColumns ? "\n\r" : ""); // -19
				delayMs(2);
				any = true;
				if (column++ == maxColumns)
					column = 1;
			}
		}
	}

	if (!any)
		print("Menu level %i empty.\r\n", menuLevel);
	else
		if (column != 1)
			print("\r\n");

	// Display errors
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++)
		if (board[deviceNumber]->errorCodeLast() != 0)
			print("Error %i in %s\n\r", board[deviceNumber]->errorCodeLast(), board[deviceNumber]->name(board[deviceNumber]->errorWasInDeviceNumber()));

	fpsPause(); // this function took too much time

	_actionCurrent = _actionDoNothing;
}

/** Color menu
*/
void Robot::menuColor() {
	menuLevel = 4;
	end();
}

/** Displays menu and stops motors
*/
void Robot::menuMainAndIdle() {
	stopAll();
	menuLevel = 1;
}

/** Reflectance menu
*/
void Robot::menuReflectance() {
	menuLevel = 2;
	end();
}

/** System menu
*/
void Robot::menuSystem() {
	menuLevel = 16;
	end();
}

/** Print CAN Bus message
@param msg - message
@param oubound - if not, inbound
*/
void Robot::messagePrint(CANBusMessage *msg, bool outbound) {
	bool any = false;
	for (uint8_t boardId = 0; boardId < _boardNextFree; boardId++) {
		if (board[boardId]->messagePrint(msg->messageId, msg->dlc, msg->data, outbound)) {
			any = true;
			break;
		}
	}
	if (!any) {
		print("Id:0x%02X", msg->messageId);
		for (uint8_t i = 0; i < msg->dlc; i++) {
			if (i == 0)
				print(" data:");
			print(" %02X", msg->data[i]);
		}
		print("\n\r");
	}
}


/** Receives CAN Bus messages. 
*/
void Robot::messagesReceive() {
	#define REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN false
	while (true) {
		_msg = mrm_can_bus->messageReceive();
		if (_msg == NULL) // No more messages
			break;
		uint32_t id = _msg->messageId;
		if (_sniff)
			messagePrint(_msg, false);
		#if REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN
		bool any = false;
		#endif
		for (uint8_t boardId = 0; boardId < _boardNextFree; boardId++) {
			// if (!strcmp(board[boardId]->name(), "US-B")){ 
			// 	print ("BRD: %s\n\r", board[boardId]->name());
			// 	messagePrint(_msg, false);
			// }
 			if (board[boardId]->messageDecode(id, _msg->data, _msg->dlc)) {
				#if REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN
				any = true;
				break;
				#endif
			}
		}

// #if REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN
// 		if (!any)
// 			print("Address device unknown: 0x%X\n\r", id);
// #endif
	}
}

/** Tests motors
*/
void Robot::motorTest() {
	print("Test motors\n\r");
	for (uint8_t i = 0; i < _boardNextFree; i++) 
		if (board[i]->boardType() == MOTOR_BOARD && board[i]->count() > 0)
			board[i]->test();
	end();
}

/** Tests mrm-node
*/
void Robot::nodeTest() {
	if (setup())
		mrm_node->start();
	mrm_node->test();
}

/** Any for or while loop must include call to this function.
*/
void Robot::noLoopWithoutThis() {
	blink(); // Keep-alive LED. Solder jumper must be shorted in order to work in mrm-esp32.
	messagesReceive();
	fpsUpdate(); // Measure FPS. Less than 30 - a bad thing.
	verbosePrint(); // Print FPS and maybe some additional data
	errors();
#if RADIO == 2
	web();
#endif
}

/** Production test
*/
void Robot::oscillatorTest() {
	if (setup()) {
		uint8_t selectedBoardIndex;
		uint8_t selectedDeviceIndex;
		uint8_t maxInput;
		uint8_t lastBoardAndDeviceIndex;
		if (boardDisplayAndSelect(&selectedBoardIndex, &selectedDeviceIndex, &maxInput, &lastBoardAndDeviceIndex)) {
			print("\n\r");
			board[selectedBoardIndex]->oscillatorTest(selectedDeviceIndex);
		}
	}
}


/**Pitch
@return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0 degrees.
*/
float Robot::pitch() {
	return mrm_imu->pitch();
}


/** Enable plug and play for all the connected boards.
 */
void Robot::pnpOn(){
	pnpSet(true);
}

/** Disable plug and play for all the connected boards.
 */
void Robot::pnpOff(){
	pnpSet(false);
}

/** Enable or disable plug and play for all the connected boards.
 @param enable - enable or disable
 */
void Robot::pnpSet(bool enable){
	uint8_t count = mrm_lid_can_b2->deadOrAliveCount();
	for (uint8_t i = 0; i < count; i++)
		if (mrm_lid_can_b2->alive(i)){
			mrm_lid_can_b2->pnpSet(enable, i);
			print("%s PnP %s\n\r", mrm_lid_can_b2->name(i), enable ? "on" : "off");
		}
	count = mrm_lid_can_b->deadOrAliveCount();
	for (uint8_t i = 0; i < count; i++)
		if (mrm_lid_can_b->alive(i)){
			mrm_lid_can_b->pnpSet(enable, i);
			print("%s PnP %s\n\r", mrm_lid_can_b->name(i), enable ? "on" : "off");
		}
			count = mrm_lid_can_b->deadOrAliveCount();
	for (uint8_t i = 0; i < count; i++)
		if (mrm_ref_can->alive(i)){
			mrm_ref_can->pnpSet(enable, i);
			print("%s PnP %s\n\r", mrm_ref_can->name(i), enable ? "on" : "off");
		}
	end();
}

/** Prints mrm-ref-can* calibration data
*/
void Robot::reflectanceArrayCalibrationPrint() {
	mrm_ref_can->calibrationDataRequest(0xFF, true);
	mrm_ref_can->calibrationPrint();
	end();
}

/** One pass of robot's program
*/
void Robot::refresh(){
		actionSet(); // Check if a key pressed and update current command buffer.
		if (_actionCurrent == NULL) // If last command finished, display menu.
			menu();
		else 
			actionProcess(); // Process current command. The command will be executed while currentCommand is not NULL. Here state maching processing occurs, too.
		noLoopWithoutThis(); // Receive all CAN Bus messages. This call should be included in any loop, like here.
}


/** Roll
@return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0�.
*/
float Robot::roll() {
	return mrm_imu->roll();
}


/** Starts robot's program
*/
void Robot::run() {
	while (true) 
		refresh();
}


/** Reads serial ASCII input and converts it into an integer
@param timeoutFirst - timeout for first input
@param timeoutBetween - timeout between inputs
@param onlySingleDigitInput - completes input after first digit
@param limit - returns 0xFFFF if overstepped
@param printWarnings - prints out of range or timeout warnings
@return - converted number or 0xFFFF when timeout
*/
uint16_t Robot::serialReadNumber(uint16_t timeoutFirst, uint16_t timeoutBetween, bool onlySingleDigitInput, uint16_t limit, bool printWarnings) {

	// Read input
	uint32_t lastMs = millis();
	uint32_t convertedNumber = 0;
	bool any = false;
	while ((millis() - lastMs < timeoutFirst && !any) || (!onlySingleDigitInput && millis() - lastMs < timeoutBetween && any)) {
		bool btAvailable = false;
#if RADIO == 1
		btAvailable = serialBT != NULL && serialBT->available();
#endif
		if (Serial.available() || btAvailable) {
			uint8_t character = 0;
			if (Serial.available())
				character = Serial.read();
#if RADIO == 1
			else if (serialBT != NULL && serialBT->available())
				character = serialBT->read();
#endif
			if (48 <= character && character <= 57) {
				convertedNumber = convertedNumber * 10 + (character - 48);
				any = true;
				lastMs = millis();
			}
		}
		noLoopWithoutThis();
	}

	// Eat tail
	while (Serial.available())
		Serial.read();
#if RADIO == 1
	while (serialBT != NULL && serialBT->available())
		serialBT->read();
#endif

	// Return result
	if (any) {
		if (convertedNumber > limit) {
			if (printWarnings)
				print("Out of range\n\r");
			return 0xFFFF;
		}
		else
			return convertedNumber;
	}
	else {
		if (printWarnings)
			print("Timeout.\n\r");
		return 0xFFFF;
	}
}

/** Moves servo motor manually
*/
void Robot::servoInteractive() {
	mrm_servo->writeInteractive();
	end();
}

/** Stops all motors
*/
void Robot::stopAll() {
	devicesStop();
	for (uint8_t i = 0; i < _boardNextFree; i++)
		if (board[i]->boardType() == MOTOR_BOARD && board[i]->count() > 0)
			((MotorBoard*)board[i])->stop();
	end();
}

/** CAN Bus stress test
*/
bool Robot::stressTest() {
	const bool STOP_ON_ERROR = true;
	const uint32_t LOOP_COUNT = 1000000;
	const bool TRY_ONLY_ALIVE = true;

	// Setup
	static uint32_t pass;
	static uint8_t lastPercent = 101;
	static uint8_t count[BOARDS_LIMIT];
	static uint32_t errors[BOARDS_LIMIT];
	static uint16_t mask[BOARDS_LIMIT]; // 16 bits - no more than 16 devices per board!

	if (setup()) {
		print("Before test.\n\r");
		pass = 0;
		devicesStop();
		for (uint8_t i = 0; i < _boardNextFree; i++)
			errors[i] = 0;
		uint8_t totalCnt = 0;
		for (uint8_t i = 0; i < _boardNextFree; i++) {
			count[i] = board[i]->devicesScan(true);
			totalCnt += count[i];
			mask[i] = TRY_ONLY_ALIVE ? 0 : 0xFFFF;
			for (uint8_t j = 0; j < board[i]->deadOrAliveCount(); j++)
				if (board[i]->alive(j))
					mask[i] |= 1 << j;
		}
		print("Start.\n\r");
		if (mrm_8x8a->alive()) {
			char buffer[50];
			sprintf(buffer, "%i devices.\n\r", totalCnt);
			mrm_8x8a->text(buffer);
			mrm_8x8a->activeCheckIfStartedSet(false); // This is not a normal state, but just not to disrupt the test.
		}
	}

	// Display progress numerically
	uint8_t percent = 100 * pass / LOOP_COUNT;
	if (percent != lastPercent && percent > 0) {
		lastPercent = percent;
		print("%i %%\n\r", percent);
	}

	// Display progress using mrm-8x8a
	if (mrm_8x8a->alive())
		if (mrm_8x8a->progressBar(LOOP_COUNT, pass, pass == 0))
			delayMs(5); // To avoid disturbing stress test

	// Stress test
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		if (count[i] > 0 || !TRY_ONLY_ALIVE) {
			delayMicros(40);
			digitalWrite(15, HIGH);
			uint8_t cnt = board[i]->devicesScan(false, mask[i]);
			digitalWrite(15, LOW);
			if (cnt != count[i]) {
				errors[i]++;
				print("***** %s: found %i, not %i.\n\r", board[i]->name(), cnt, count[i]);
				if (STOP_ON_ERROR) {
					if (mrm_8x8a->alive()) {
						char buffer[50];
						sprintf(buffer, "%s: error.\n\r", board[i]->name());
						mrm_8x8a->text(buffer);
					}
					pass = LOOP_COUNT - 1;
					break;
				}
			}
		}
	}

	// Results
	if (++pass >= LOOP_COUNT || userBreak()) {
		bool allOK = true;
		for (uint8_t i = 0; i < _boardNextFree; i++)
			if (count[i] > 0 && errors[i] > 0) {
				print("%s: %i errors.\n\r", board[i]->name(), errors[i]);
				allOK = false;
				delay(5000); // To freeze oscilloscope
			}
		if (allOK) {
			if (mrm_8x8a->alive())
				mrm_8x8a->bitmapDisplay(0);
			print("No errors.");
		}
		print("\n\r");
		end();
		return true;
	}
	else
		return false;
}

/** Tests mrm-therm-b-can
*/
void Robot::thermoTest() {
	if (setup())
		mrm_therm_b_can->start();
	mrm_therm_b_can->test();
}

/** Checks if user tries to break the program
@return - true if break requested.
*/
bool Robot::userBreak() {
	bool btAvailable = false;
#if RADIO == 1
	btAvailable = serialBT != NULL && serialBT->available();
#endif
	if (/*switchOn() ||*/ Serial.available() || btAvailable) {
		return true;
	}
	else
		return false;
}

/** Prints additional data in every loop pass
*/
void Robot::verbosePrint() {
	if (verbose) {
		static uint32_t lastMs = 0;
		if (lastMs == 0 || millis() - lastMs > 5000) {
			print("%i fps\r\n", (uint16_t)fpsGet());
			lastMs = millis();
		}
	}
}

/** Verbose output toggle
*/
void Robot::verboseToggle() {
	verbose = !verbose;
};

#if RADIO == 2
/** Web server
*/
void Robot::web(){
	static uint32_t previousTime = 0;
	static uint32_t currentTime = 0;
	const uint16_t timeoutTime = 2000;
	// Variable to store the HTTP request
	String header;

	WiFiClient client = webServer->available();   // Listen for incoming clients
   


  	if (client) {                             // If a new client connects,
		currentTime = millis();
		previousTime = currentTime;
		print("New Client.\n\r");          // print a message out in the serial port
		String currentLine = "";                // make a String to hold incoming data from the client
		while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
			currentTime = millis();
			if (client.available()) {             // if there's bytes to read from the client,
				char c = client.read();             // read a byte, then
				print("%c", c);                    // print it out the serial monitor
				header += c;
				if (c == '\n') {                    // if the byte is a newline character
					// if the current line is blank, you got two newline characters in a row.
					// that's the end of the client HTTP request, so send a response:
					if (currentLine.length() == 0) {
						// HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
						// and a content-type so the client knows what's coming, then a blank line:
						client.println("HTTP/1.1 200 OK");
						client.println("Content-type:text/html");
						client.println("Connection: close");
						client.println();
						
						// Display the HTML web page
						client.println("<!DOCTYPE html><html>");
						client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
						client.println("<link rel=\"icon\" href=\"data:,\">");
						// CSS to style the on/off buttons 
						// Feel free to change the background-color and font-size attributes to fit your preferences
						client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
						client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
						client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
						client.println(".button2 {background-color: #555555;}</style></head>");
						
						// Web Page Heading
						client.println("<body><h1>ESP32 Web Server</h1>");
						
						// Display current state, and ON/OFF buttons for GPIO 26  
						client.println("<p>State </p>");

						client.println("</body></html>");
						
						// The HTTP response ends with another blank line
						client.println();
						// Break out of the while loop
						break;
					} else { // if you got a newline, then clear currentLine
						currentLine = "";
					}
				} else if (c != '\r') {  // if you got anything else but a carriage return character,
					currentLine += c;      // add it to the end of the currentLine
				}
			}
		}
		// Clear the header variable
		header = "";
		// Close the connection
		client.stop();
		Serial.println("Client disconnected.");
		Serial.println("");
	}
}
#endif