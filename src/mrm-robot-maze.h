#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

enum Direction{UP, LEFT, DOWN, RIGHT, NOWHERE};
enum LedSign { MAZE_IMU_FOLLOW, MAZE_LED_PAUSE, MAZE_LED_PLAY, MAZE_WALL_DOWN_FOLLOW, MAZE_WALL_LEFT_FOLLOW, MAZE_WALL_RIGHT_FOLLOW, MAZE_WALL_UP_FOLLOW }; // For mrm-8x8 display.
enum WallStatus{ NO_WALL, WALL_UNKNOWN, WALL_WITH_VICTIM, WALL_WITHOUT_VICTIM};
enum TileSurvace{ SURFACE_BLACK, SURFACE_SILVER, SURFACE_UNKNOWN, SURFACE_WHITE};


/** Class Tile stores data for tiles: walls, etc. The whole maze consists of one single-linked chain of Tile objects, chained using _chain pointers.
*/
class Tile {

	uint8_t _wall; // 1 byte stores statuses for all 2 walls enclosing the tile. Bit 7:6 - right, 5:4 - down, 3:2 - left, 1:0 - up.
	uint8_t _surface; // 1 byte stores surface and other data. Bit 7:6 - surface. The rest 6 bytes can be freely used. This feature is not in use yet.
	Tile* _chain; // Pointer to the next tile in chain.

public:
	static Tile* first; // Static member variable (one instance for the whole class), pointing to the first tile of the chain.

	Direction breadcrumb; // Stores direction of the previous tile, needed for Tremaux algorithm. This sequence is independent of the one made by _chain variable, which is only used for traversing the chain.
	int8_t x; // x coordinate of the tile. Increasing in RIGHT direction.
	int8_t y; // y coordinate of the tile. Increasing in UP direction.

	/** Constructor
	@param x - x coordinate.
	@param y - y coordinate.
	@param breadcrumb - way back to the start tile.
	*/
	Tile(int8_t x, int8_t y, Direction breadcrumb); 

	/** Enables traversing the chain from outside of the class.
	@return - link to the next tile in chain.
	*/
	Tile* next() { return _chain; } 

	/** Existence of the wall, victims, etc.
	@param direction - direction of the wall.
	@return status.
	*/
	WallStatus wallGet(Direction direction) { return (WallStatus)((_wall >> (direction * 2)) & 0b11); }

	/** Sets wall's status.
	@param direction - direction of the wall.
	@param status - new status.
	*/
	void wallSet(Direction direction, WallStatus wallStatus) { _wall &= ~(0b11 << (direction * 2)); _wall |= (wallStatus << (direction * 2)); }
};


/* All the Action-classes have to be forward declared here (before RobotMaze) as RobotMaze declaration uses them. The other option would be
not to declare them here, but in that case Action-objects in RobotMaze will have to be declared as ActionBase class, forcing downcast later in code, if
derived functions are used.*/
class ActionDecide;
class ActionMap;
class ActionMove;
class ActionMoveAhead;
class ActionMoveTurn;
class ActionRescueMaze;
class ActionWallsTest;

/** Robot for RCJ Rescue Maze, a class derived from the base Robot class.
*/
class RobotMaze : public Robot {
	// Change these values to get optimal robot's behaviour.
	const uint16_t IMU_FOLLOW_STRENGTH = 5; // A bigger value increases feedback.
	const uint16_t MAZE_MAX_STEPS = 0xFFFF; // Can be used to stop the robot after a certain number of steps, for example for debugging purpose.
	const uint16_t MOVE_AHEAD_TIMEOUT_MS = 1300; // If no encoders available, the robot will stop after this time elapses, when moving ahead over 1 tile.
	const uint16_t NO_WALL_DISTANCE = 250; // In mm. When a sensor measures a value bigger than this one, it will declare there is no wall in front of it.
	const uint8_t TOP_SPEED = 127; // Can be used to slow down the robot. Maximum value is 127. Enter 0 to test the robot without motors working.
	const uint16_t WALL_FOLLOW_DISTANCE = 90; // Distance for wall following. Normally it should be (300 mm - 120 mm) / 2. 300 mm is tile's width and 120 mm is robot's width.
	const uint16_t WALL_FOLLOW_ERROR_ALLOWED = 40; // In mm. If 2 sensors measure distances that diverge in values more than this number, the robot will not follow that wall.
	const uint16_t WALL_FOLLOW_DISTANCE_ADJUSTMENT_STRENGTH = 1.1; // A bigger value will force the robot to correct distance to a wall more vigorously.
	const uint16_t WALL_FOLLOW_ROTATION_STRENGTH = 1.1; // A bigger value will force the robot to correct its alignment to a wall more vigorously.

	// Actions' declarations
	ActionDecide* actionDecide;
	ActionBase* actionGoStraightAhead;
	ActionMap* actionMap;
	ActionMove* actionMove;
	ActionMoveAhead* actionMoveAhead;
	ActionMoveTurn* actionMoveTurn;
	ActionRescueMaze* actionRescueMaze;
	ActionWallsTest* actionWallsTest;

	Direction directionCurrent; // Current robot's direction.

	float imuLastValid = 9999; // Last measured compass value. Important for moving ahead when no wall available.

	MotorGroupDifferential* motorGroup = NULL; // Class that conveys commands to motors.

	uint16_t stepCount; // Number of steps made.

	Tile* tileCurrent; // Current tile (robot's position).

public:
	/** Constructor
	@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
	*/
	RobotMaze(char name[] = "RCJ Maze");

	/** Stores custom bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	/** Function that decides what to do next, using Tremaux algorithm. If a not-visited direction exists, go there. If not, return to the tile robot came from.
	*/
	void decide();

	/** Distance measuring function. Sensors must have deviceId addresses exactly like here: 0 FR, 1 FL, 2 LF, 3 LB, 4 BL, 5 BR, 6 RB, 7 RF. F is front, B is back, L is left, R is right. 
	In 2-letter designations first letter is a major direction, second minor. For example FL means Front-Left and first we choose major side (robot's front), then minor (left sensor on that side). 
	In other words, first sensor is front-right and all the others must have increasing	addresses counter-clockwise (CCW).
	@direction - direction in maze's system. Therefore, LEFT is always left, as seen from outside of the maze, no matter what the robot's direction is. It is normally not robot's left side.
	@firstCCW - first when counting ounter-clockwise (CCW) on that side, looking from inside of the robot. For example FL is first CCW and FR is not.
	@return - distance in mm.
	*/
	uint16_t distance(Direction direction, bool firstCCW) {return mrm_lid_can_b->reading(2 * mToR(direction) + firstCCW); }

	/** Displays direction in human-readable format. For debugging purposes.
	@param - direction in maze's system.
	*/
	void directionDisplay(Direction direction);

	/** Orders the robot to go ahead. Overriden virtual function. Not used here.
	*/
	void goAhead() {}

	/** Drives the robot ahead, maintaing a given compass bearing.
	*/
	void imuFollow();

	/** Custom test
	*/
	void loop();

	/** Maps walls detected and other external readings in variables.
	*/
	void map();

	/** Displays the whole maze.
	*/
	void mazePrint();

	/** Moves robot, either forward (moveAhead()) or by turning it (moveTurn()).
	*/
	void move();

	/** Drives the robot straight ahead.
	*/
	void moveAhead();

	/** Turns the robot till the target bearing achieved.
	*/
	void moveTurn();

	/* Direction from maze's perspective to robot's perspective. Robot's front is TOP, robot's back is DOWN.
	@param directionAsSeenInMaze - direction from maze's prespective.
	@return direction from robot's perspective.
	*/
	Direction mToR(Direction directionAsSeenInMaze) { int8_t d = directionAsSeenInMaze - directionCurrent; return (Direction) (d < 0 ? d + 4 : d); }

	/** Test for Mecanum wheels. Used only when the robot is rigged with mecanum wheels.
	*/
	void omniWheelsTest();

	/** Starts RCJ Rescue Maze run.
	*/
	void rescueMaze();

	/* Direction from robot's perspective to maze's perspective. Robot's front is TOP, robot's back is BACK.
	@param directionAsSeenByRobot - direction from robot's perspective.
	@return direction from maze's perspective.
	*/
	Direction rToM(Direction directionAsSeenByRobot) { return (Direction)(directionCurrent + directionAsSeenByRobot % 4); }

	/** Traverses all the chain till a tile with (x,y) is found.
	@param x - x coordinate.
	@param y - y coordinata.
	@return - a tile with given coordinates and NULL if none exists.
	*/
	Tile* tileContaining(int8_t x, int8_t y);

	/** Finds the closest of the 4 walls around the tile.
	@return - direction from maze's perspective.
	*/
	Direction wallClosest();

	/** Goes ahead by following a wall.
	@wallDirection - direction in maze's perspective.
	*/
	void wallFollow(Direction wallDirection);

	/** Uses 8x8 LED to display walls detected.
	*/
	void wallsDisplay();

	/** Checks if wall exists in tile (x,y) and direction direction.
	@param x - tile's x coordinate.
	@param y - tile's y coordinate.
	@param direction - wall's direction.
	@return - type of wall.
	*/
	WallStatus wallGet(int8_t x, int8_t y, Direction direction);

	/** Display one wall.
	@param wallStatus - wall's status.
	@param direction - direction.
	*/
	void wallDisplay(WallStatus wallStatus, Direction direction);

	/** Test, checking and displaying all walls.
	*/
	void wallsTest();

	/** x coordinate of the tile next to the current one, in a given direction.
	@param direction.
	@return x coordinate.
	*/
	int8_t x(Direction direction);

	/** y coordinate of the tile next to the current one, in a given direction.
	@param direction.
	@return y coordinate.
	*/
	int8_t y(Direction direction);
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

/** Actions specific for a RobotMaze robot will be defined here. They are all derived from ActionBase class (inheriting its methods and properties). 
They also all feature perform() function, the one that is called to do what they are supposed to. Their constructors always call base class' constructors [ActionBase(...)] with 3 or more 
parameters specified here.
First parameter is robot and is always the same. 
The second one is a 3-letter shortcut that is displayed in command menu. For example "maz" will be displayed for starting the maze run. When action is not supposed to be started from menu, 
it can be an empty string.
The third parameter is a name of the action, again displayed in menu. For "maz", the name is "Rescue Maze", causing menu entry "maz - Rescue Maze" to be displayed. Again, use empty string 
for no-menu actions.
The fourth pareameter is menu level. When omitted, the action will not be a part of the menu. Use 1 otherwise. Higher level numbers will display the action in submenues, not used here.
*/

/** Action that decides what to do next, using Tremaux algorithm. If a not-visited direction exists, go there. If not, return to the tile robot came from.
*/
class ActionDecide : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->decide(); }
public:
	ActionDecide(Robot* robot) : ActionBase(robot, "", "") {}
};

/** Maps walls detected and other external readings in variables.
*/
class ActionMap : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->map(); }
public:
	ActionMap(Robot* robot) : ActionBase(robot, "", "") {}
};

/** Base class for movement actions.
*/
class ActionMove : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->move(); }
public:
	Direction direction; // Initial direction is stored to be recalled when using the action.
	uint32_t startMs; // Time is stored, for example to be used to determine if the movement's timeout elapsed.
	ActionMove(Robot* robot) : ActionBase(robot, "", "") {}
};

/** Go straight ahead.
*/
class ActionMoveAhead : public ActionMove {
	void perform() { ((RobotMaze*)_robot)->moveAhead(); }
public:
	ActionMoveAhead(Robot* robot) : ActionMove(robot) {}
};

/** Turn.
*/
class ActionMoveTurn : public ActionMove {
	void perform() { ((RobotMaze*)_robot)->moveTurn(); }
public:
	float endAngle; // End condition: target angle.
	float turnByCCW; // Number of degrees to turn CCW (counter-clockwise).
	ActionMoveTurn(Robot* robot) : ActionMove(robot) {}
};

/** Start RCJ Rescue Maze run.
*/
class ActionRescueMaze : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->rescueMaze(); }
public:
	ActionRescueMaze(Robot* robot) : ActionBase(robot, "maz", "Rescue Maze", 1) {}
};

/** Test for Mecanum wheels. Used only when the robot is rigged with mecanum wheels.
*/
class ActionOmniWheelsTest : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->omniWheelsTest(); }
public:
	ActionOmniWheelsTest(Robot* robot) : ActionBase(robot, "omn", "Test omni wheels", 1) {}
};

/** Test, checking and displaying all walls.
*/
class ActionWallsTest : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->wallsTest(); }
public:
	ActionWallsTest(Robot* robot) : ActionBase(robot, "wlt", "Walls test", 1) {}
};