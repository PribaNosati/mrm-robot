#pragma once
#include <mrm-action.h>
#include <mrm-pid.h>
#include <mrm-robot.h>

/** Robot for RCJ Soccer
*/
class RobotSoccer : public Robot {

	// Actions' declarations
	ActionBase* actionCatch;
	ActionBase* actionIdle;

	float headingToMaintain; // Heading towards opponent's goal.
	MotorGroupStar* motorGroup;  // Class that conveys commands to motors.
	Mrm_pid* pidXY; // PID controller, regulates motors' speeds for linear motion in the x-y plane.
	Mrm_pid* pidRotation; // PID controller, regulates rotation around z axis.
public:
	/** Constructor
	@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
	*/
	RobotSoccer(char name[] = "RCJ Soccer");

	/** Read barrier
	@return - true if interrupted
	*/
	bool barrier();

	/** Store bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	/** Go around the ball and approach it.
	*/
	void catchBall();

	/** Front distance to wall
	@return - in mm
	*/
	uint16_t front();

	/** Test - go straight ahead.
	*/
	void goAhead();

	/** No ball detected - return to Your goal.
	*/
	void idle();

	/** Front distance to wall
	@return - in mm
	*/
	uint16_t left();

	/** Custom test
	*/
	void loop();

	/** Starts robot.
	*/
	void play();

	/** Rear distance to wall
	@return - in mm
	*/
	uint16_t rear();

	/** Right distance to wall
	@return - in mm
	*/
	uint16_t right();
};

/** Actions serve a few purposes.
- They encapsulate in classes actions robot has to perform. So, we have classes for robot's parts, but here also for non-material terms.
- No global variables are used. When an information should be shared between one (but called repeatedly) or more functions, it will be stored inside the action object. For example, all the
start conditions will be in the object itself.
- You can use inheritance to indicate relationships between actions, which indeed exist. For example, a movement can be movement straight ahead or turning.
- You can use in a consistent way actions defined for the base robot, without its code being exposed here.
- The actions are included in menus just by including a parameter in the constructor call.
- Buttons can be used to start actions, as well as menu commands. Menus are displayed both in the connected PC and a Bluetooth device, like a mobile phone, and any of the 2 can be used to
issue commands.
*/

/** Actions specific for a RobotSoccer robot will be defined here. They are all derived from ActionBase class (inheriting its methods and properties).
They also all feature perform() function, the one that is called to do what they are supposed to. Their constructors always call base class' constructors [ActionBase(...)] with 3 or more
parameters specified here.
First parameter is robot and is always the same.
The second one is a 3-letter shortcut that is displayed in command menu. For example "soc" will be displayed for starting the RCJ Soccer run. When action is not supposed to be started from menu,
it can be an empty string.
The third parameter is a name of the action, again displayed in menu. For "soc", the name is "Soccer play", causing menu entry "soc - Soccer play" to be displayed. Again, use empty string
for no-menu actions.
The fourth pareameter is menu level. When omitted, the action will not be a part of the menu. Use 1 otherwise. Higher level numbers will display the action in submenues, not used here.
*/

/** Go around the ball and approach it.
*/
class ActionSoccerCatch : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->catchBall(); }
public:
	ActionSoccerCatch(RobotSoccer* robot) : ActionBase(robot, "cat", "Soccer catch", 0) {}
};

/** Starts robot.
*/
class ActionSoccerPlay : public ActionBase {
	void perform(){ ((RobotSoccer*)_robot)->play(); }
public:
	ActionSoccerPlay(RobotSoccer* robot) : ActionBase(robot, "soc", "Soccer play") {}
};

/** Idle position, before own goal. 
*/
class ActionSoccerIdle : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->idle(); }
public:
	ActionSoccerIdle(RobotSoccer* robot) : ActionBase(robot, "idl", "Soccer idle", 0) {}
};
