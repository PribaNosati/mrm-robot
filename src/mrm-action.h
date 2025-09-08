#pragma once
#include <Arduino.h>
#include <mrm-board.h>
#include <mrm-ref-can.h>
#include <mrm-8x8a.h>

class Robot;
class ActionBase {
protected:
	Board::BoardId _boardsId;
	bool _preprocessing = true;
	Robot* _robot;
	void (Robot::*_actionPerform)();

public:
	char _text[20];
	uint8_t _menuLevel;
	Mrm_8x8a::LEDSign* ledSign;

	/** Constructor
	@param robot - robot
	@param text - menu entry
	@param menuLevel - all the actions with the same menuLevel are displayed. Permitted values are: 0 (in no menu), 1, 2, 4, 16, 16, 32, 64, and 128. 
		A menu-action changes menuLevel to its own, forcing all the actions with this menuLevel to be displayed. "|" lists action in many menus, for example 1 | 16 | 16.
	@param boardId - menu only for a specific board
	@param ledSign - the LED sign that will be displayed when action set to this one
	*/
	ActionBase(Robot* robot, const char text[20], uint8_t menuLevel = 1, Board::BoardId boardsId = Board::BoardId::ID_ANY,
		Mrm_8x8a::LEDSign* ledSign8x8 = NULL, void (Robot::*actionPerform)() = NULL);

	Board::BoardId boardsId() { return _boardsId; }

	bool preprocessing() { return _preprocessing; }

	void preprocessingEnd() { _preprocessing = false; }

	void preprocessingStart() { _preprocessing = true; }

	virtual void performAfter() {}; // todo, not implemented.

	virtual void performBefore() {};

	virtual void perform() = 0;
};

class ActionRobot : public ActionBase {
	void perform();
public:
	ActionRobot(Robot* robot, const char text[20], uint8_t menuLevel = 1, Board::BoardId boardsId = Board::BoardId::ID_ANY,
		Mrm_8x8a::LEDSign* ledSign8x8 = NULL,  void (Robot::*actionPerform)() = NULL) : 
		ActionBase(robot, text, menuLevel, boardsId, ledSign8x8, actionPerform) {}
};