#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

/* All the Action-classes have to be forward declared here (before RobotMin) as RobotMin declaration uses them. The other option would be
not to declare them here, but in that case Action-objects in RobotMin will have to be declared as ActionBase class, forcing downcast later in code, if
derived functions are used.*/


/** Minimum robot, a class derived from the base Robot class.
*/
class RobotMin : public Robot {

	MotorGroupDifferential* motorGroup = NULL; // Class that conveys commands to motors.

	// Actions' declarations


public:
	/** Constructor
	@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
	*/
	RobotMin(char name[] = "RCJ Min");

	/** Stores custom bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet() {}

	/** Test - go straight ahead using a defined speed.
	*/
	void goAhead(){}

	/** Custom test
	*/
	void loop();
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

/** Actions specific for a RobotMin robot will be defined here. They are all derived from ActionBase class (inheriting its methods and properties). 
They also all feature perform() function, the one that is called to do what they are supposed to. Their constructors always call base class' constructors [ActionBase(...)] with 3 or more 
parameters specified here.
First parameter is robot and is always the same. 
The second one is a 3-letter shortcut that is displayed in command menu. For example "maz" will be displayed for starting the maze run. When action is not supposed to be started from menu, 
it can be an empty string.
The third parameter is a name of the action, again displayed in menu. For "maz", the name is "Rescue Maze", causing menu entry "maz - Rescue Maze" to be displayed. Again, use empty string 
for no-menu actions.
The fourth pareameter is menu level. When omitted, the action will not be a part of the menu. Use 1 otherwise. Higher level numbers will display the action in submenues, not used here.
*/

