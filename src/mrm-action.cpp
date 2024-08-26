#include <mrm-action.h>

/** Constructor
@param robot - robot
@param shortcut - up-to-3-letter word
@param text - menu entry
@param menuLevel - all the actions with the same menuLevel are displayed. Permitted values are: 0 (in no menu), 1, 2, 4, 8, 16, 32, 64, and 128. 
	A menu-action changes menuLevel to its own, forcing all the actions with this menuLevel to be displayed. "|" lists action in many menus, for example 1 | 8 | 16.
@param boardId - menu only for a specific board
@param ledSign8x8 - the LED sign that will be displayed when action set to this one
*/
ActionBase::ActionBase(Robot* robot, const char shortcut[4], const char text[20], uint8_t menuLevel, Board::BoardId boardsId,
	Mrm_8x8a::LEDSign* ledSign8x8, void (Robot::*actionPerform)()) {
	_robot = robot;
	if (shortcut != 0)
		strcpy(_shortcut, shortcut);
	if (text != 0)
		strcpy(_text, text);
	_menuLevel = menuLevel;
	_boardsId = boardsId;
	ledSign = ledSign8x8;
	_actionPerform = actionPerform;
}

void ActionRobot::perform() { if (_actionPerform != NULL)  (_robot->*_actionPerform)(); }