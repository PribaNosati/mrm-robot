#include <mrm-8x8a.h>
#include <mrm-col-can.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot4x3.6can.h>
#include "mrm-robot-maze.h"
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>

Tile* Tile::first; // Neccessary for static member variable.

/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotMaze::RobotMaze(char name[]) : Robot(name) {
	// MotorGroup class drives the motors.
	// 2nd, 4th, 6th, and 8th parameters are output connectors of the controller (0 - 3, meaning 1 - 4. connector). 2nd one must be connected to LB (Left-Back) motor,
	// 4th to LF (Left-Front), 6th to RF (Right-Front), and 8th to RB (Right-Back). Therefore, You can connect motors freely, but have to
	// adjust the parameters here. In this example output (connector) 3 is LB, etc.
	motorGroup = new MotorGroupDifferential(this, mrm_mot4x3_6can, 3, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 0);

	// Depending on your wiring, it may be necessary to spin some motors in the other direction. In this example, no change needed,
	// but uncommenting the following line will change the direction of the motor 2.
	//mrm_mot4x3_6can->directionChange(2);

	// All the actions will be defined here; the objects will be created.
	actionDecide = new ActionDecide(this);
	actionGoStraightAhead = new ActionMoveAhead(this);
	actionMap = new ActionMap(this);
	actionMove = new ActionMove(this);
	actionMoveAhead = new ActionMoveAhead(this);
	actionMoveTurn = new ActionMoveTurn(this);
	actionRescueMaze = new ActionRescueMaze(this);

	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus. For example, ActionWallsTest test is only manu-based, and it is all right. 
	// This test is not supposed to be called in code.
	actionAdd(actionRescueMaze);
	actionAdd(new ActionOmniWheelsTest(this));
	actionAdd(new ActionWallsTest(this));

	// Set buttons' actions
	mrm_8x8a->actionSet(actionRescueMaze, 0); // Button 0 starts RCJ Maze.
	// Put Your buttons' actions here.

	// Upload custom bitmaps into mrm-8x8a.
	bitmapsSet();
}

/** Store custom bitmaps in mrm-led8x8a.
*/
void RobotMaze::bitmapsSet() {
	mrm_8x8a->alive(0, true); // Makes sure that mrm-8x8a is present and functioning. If not, issues a warning message.

	// The 2 arrays will hold red and green pixels. Both red an green pixel on - orange color.
	uint8_t red[8];
	uint8_t green[8];
	for (uint8_t i = 0; i < 8; i++) // Erase all
		red[i] = 0, green[i] = 0;

	/* 1 will turn the pixel on, 0 off. 0bxxxxxxxx is a binary format of the number. Start with "0b" and list all the bits, starting from 
	the most significant one (MSB). Do that for each byte of the green and red arrays.*/

	// Pause
	green[0] = 0b11100111;
	green[1] = 0b11100111;
	green[2] = 0b11100111;
	green[3] = 0b11100111;
	green[4] = 0b11100111;
	green[5] = 0b11100111;
	green[6] = 0b11100111;
	green[7] = 0b11100111;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	/* Store this bitmap in mrm-8x8a. The 3rd parameter is bitmap's address. If You want to define new bitmaps, expand LedSign enum with
	Your names, and use the new values for Your bitmaps. This parameter can be a plain number, but enum keeps thing tidy.*/
	mrm_8x8a->bitmapCustomStore(red, green, LedSign::MAZE_LED_PAUSE); 
	delayMs(1); // Wait a little in order not to queue too many CAN Buss messages at once.

	// Play.
	green[0] = 0b0110000;
	green[1] = 0b0111000;
	green[2] = 0b0111100;
	green[3] = 0b0111110;
	green[4] = 0b0111110;
	green[5] = 0b0111100;
	green[6] = 0b0111000;
	green[7] = 0b0110000;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LedSign::MAZE_LED_PLAY);
	delayMs(1);

	// Follow IMU.
	green[0] = 0b00000000;
	green[1] = 0b00010000;
	green[2] = 0b00111000;
	green[3] = 0b01111100;
	green[4] = 0b00111000;
	green[5] = 0b00111000;
	green[6] = 0b00000000;
	green[7] = 0b00000000;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LedSign::MAZE_IMU_FOLLOW);
	delayMs(1);

	// Follow wall down, green taken from Follow IMU bitmap.
	for (uint8_t i = 0; i < 7; i++)
		red[i] = 0;
	red[7] = 0b11111111;
	mrm_8x8a->bitmapCustomStore(red, green, LedSign::MAZE_WALL_DOWN_FOLLOW);
	delayMs(1);

	// Follow wall left, green taken from Follow IMU bitmap.
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0b10000000;
	mrm_8x8a->bitmapCustomStore(red, green, LedSign::MAZE_WALL_LEFT_FOLLOW);
	delayMs(1);

	// Follow wall right, green taken from Follow IMU bitmap.
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0b00000001;
	mrm_8x8a->bitmapCustomStore(red, green, LedSign::MAZE_WALL_RIGHT_FOLLOW);
	delayMs(1);

	// Follow wall up, green taken from Follow IMU bitmap.
	red[0] = 0b11111111;
	for (uint8_t i = 1; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LedSign::MAZE_WALL_UP_FOLLOW);
	delayMs(1);

	// Define Your bitmaps here.

	delayMs(10); // Wait a little to be sure that the next sign will be displayed.
}


/** Function that decides what to do next, using Tremaux algorithm. If a not-visited direction exists, go that route. 
If not, return to the tile robot came from.
*/
void RobotMaze::decide(){
	motorGroup->stop();
	wallsDisplay(); // Use mrm-8x8a to display walls.
	actionMove->direction = Direction::NOWHERE; // Initialize direction.
	for (uint8_t i = 0; i < 4; i++) {
		Direction dir = (Direction)i; // An integer (i) can be casted to enum (Directory).
		/* If there is no wall in this direction, and there is no tile in chain with coordinates that correspond to that adjacent tile (i.e. it
		has not been visited yet), that means that this is a free passage that can be chosen to go on.*/
		if (tileCurrent->wallGet(dir) == WallStatus::NO_WALL && tileContaining(x(dir), y(dir)) == NULL) { // NULL - not visited
			actionSet(actionMove); // The next action will be moving ahead.
			actionMove->direction = dir; // The next direction will be the one that leaeds to a free passage (dir).
			print("Not visited direction found: "); // Debug info.
			directionDisplay(dir); // Debug info.
			print("\n\r"); // Debug info.
			/* Break the for loop. Note that this instruction significantly defines robot's behavior. Like this, it will always choose
			the first free tile in the direction of the "for" loop (CCW). The direction of the loop could be changed to CW or some other strategy
			could be chosen, like "always go straight, if possible".*/
			break; 
		}
	}
	if (actionMove->direction == Direction::NOWHERE) // No direction found, so go back using breadcrumbs.
		if (tileCurrent->breadcrumb == Direction::NOWHERE) {//We are on the first tile and there is no way back - end of run.
			end(); // This command will cancel actions and the robot will return to the default idle loop, after displaying menu.
			mrm_8x8a->bitmapCustomStoredDisplay(LedSign::MAZE_LED_PAUSE); // Display pause sign.
			print("END\n\r");
		}
		else { // A way back found. Retreat in that direction.
			print("Go back\n\r");
			actionMove->direction = tileCurrent->breadcrumb; // Set the new direction to go back.
			actionSet(actionMove); // The next action will be moving ahead (although in the logically backward-direction).
		}

	//print("cnt=%i\n\r", stepCount);
	
	// If the robot should stop after a certain number of steps (for example for debugging), do it here.
	if (stepCount++ >= MAZE_MAX_STEPS) {
		motorGroup->stop();
		end(); // This command will cancel actions and the robot will return in the default idle loop, after displaying menu.
	}
}


/** Displays direction in human-readable format. For debugging purposes.
@param - direction in maze's system.
*/
void RobotMaze::directionDisplay(Direction direction){
	switch (direction) {
	case Direction::DOWN:
		print("down");
		break;
	case Direction::UP:
		print("up");
		break;
	case Direction::LEFT:
		print("left");
		break;
	case Direction::RIGHT:
		print("right");
		break;
	case Direction::NOWHERE:
		print("nowhere");
		break;
	}
}

/** Drives the robot ahead, maintaing a given compass bearing.
*/
void RobotMaze::imuFollow() {
	mrm_8x8a->bitmapCustomStoredDisplay(LedSign::MAZE_IMU_FOLLOW); // Display IMU-following sign.
	float errorCW = imuLastValid - mrm_imu->heading(); // Calculate rotational error in clockwise direction.
	int16_t slowerMotor = TOP_SPEED - errorCW * IMU_FOLLOW_STRENGTH; // Calculate slower motor's speed in order to adjust heading proportionally to error.
	motorGroup->go(errorCW > 0 ? slowerMotor : TOP_SPEED, errorCW < 0 ? TOP_SPEED : slowerMotor); // Drive the robot.
}

/** Custom test. The function will be called many times during the test, till You issue "x" menu-command.
*/
void RobotMaze::loop() {
	if (setup()) { // This part will execute only in the firs run.
		directionCurrent = Direction::LEFT;
		mrm_8x8a->rotationSet(LED_8X8_BY_90_DEGREES);
	}

	directionDisplay(wallClosest());
	print("\n\r");
	delay(200);
}

/** Maps walls detected and other external readings in variables.
*/
void RobotMaze::map() {
	for (uint8_t i = 0; i < 4; i++) { // Iterate the 4 directions.
		Direction dir = (Direction)i; // An integer (i) can be casted to enum (Directory).
		if (tileCurrent->wallGet(dir) == WallStatus::WALL_UNKNOWN) // Map only the walls not mapped before (that have status WALL_UNKNOWN).
			/*If both sensors in this direction detect walls, conclude that a wall is ahead. Note that this is not a straightforward decision.
			The alternative strategy would be to declare a wall if ANY of the sensors detects a wall and it would be more appropriate for the 
			situation when the robot has not gone far enough and its rear sensor is missing the wall, which is only 1 cm ahead. The third strategy
			would be to consider a more elaboraty solution. The last one is the best.
			*/
			if (distance(dir, false) < NO_WALL_DISTANCE && distance(dir, true) < NO_WALL_DISTANCE)
				tileCurrent->wallSet(dir, WallStatus::WALL_WITHOUT_VICTIM); // Map the wall.
			else
				tileCurrent->wallSet(dir, WallStatus::NO_WALL); // Map empty space.
	}
	actionSet(actionDecide); // After mapping the next action will be a decision what to do.
	mazePrint(); // A debug tool: print all the maze after each map event.
}

/** Displays the whole maze.
*/
void RobotMaze::mazePrint() {
	// First find minimum and maximum for both coordinates.
	int8_t minX = 127;
	int8_t maxX = -128;
	int8_t minY = 127;
	int8_t maxY = -128;
	for (Tile* tile = Tile::first; tile != NULL; tile = tile->next()) {
		if (tile->x > maxX)
			maxX = tile->x;
		if (tile->x < minX)
			minX = tile->x;
		if (tile->y > maxY)
			maxY = tile->y;
		if (tile->y < minY)
			minY = tile->y;
	}
	//print("(%i, %i) - (%i, %i)\n\r", minX, maxX, minY, maxY);
	//print("Wall right: %i, _wall: %i %i\n\r", wallGet(0, 1, Direction::RIGHT), Tile::first->_wall, tileCurrent->_wall);

	// Print the maze line by line, starting with its top (max. y). The only way beacuse of the scrolling nature of a terminal (text) screen.
	for (int16_t y = maxY; y >= minY; y--) {
		// Upper border. When on the top (max. y), first display the top border. Afterwards display in each step only bottom walls.
		if (y == maxY) {
			print("┌"); // UTF-8 sign. Note that You must have an UTF-8 capable terminal. Otherwise, this sign will not show.
			for (int16_t x = minX; x <= maxX; x++) {
				wallDisplay(wallGet(x, y, Direction::UP), Direction::UP); // Display a wall or whatever its status is, like no-wall.
				print(x == maxX ? "┐" : "┬"); // Always print crosses between corner of the tiles.
			}
			print("\n\r"); // Upper border printed, end the line.
		}

		// Left border. For each row, first print its left wall. Then for, each tile, its right wall.
		wallDisplay(wallGet(minX, y, Direction::LEFT), Direction::LEFT);

		// Right walls of each tile
		for (int16_t x = minX; x <= maxX; x++) {
			print(" ");
			wallDisplay(wallGet(x, y, Direction::RIGHT), Direction::RIGHT);
		}
		print("\n\r"); // This completes verticall middle for this row. The next are bottom walls.

		// Bottom walls
		print(y == minY ? "└" : "├");
		for (int16_t x = minX; x <= maxX; x++) {
			wallDisplay(wallGet(x, y, Direction::DOWN), Direction::DOWN);
			print(x == maxX ? (y == minY ? "┘" : "┤") : (y == minY ? "┴" : "┼"));
		}
		print("\n\r");
	}
}

/** Moves robot, either forward (moveAhead()) or by turning it (moveTurn()).
*/
void RobotMaze::move() {
	if (actionMove->direction == directionCurrent) { // The robot is already in position where the next free way is right in front of it. Just go ahead.
		print("Ahead\n\r");
		actionSet(actionMoveAhead); // Set the next action: go straight ahead.
		actionMoveAhead->startMs = millis(); // Mark the start time.
	}
	else{ // The robot's front is not oriented towards the next free. Therefore, first rotate the robot.
		int16_t delta = actionMove->direction - directionCurrent; // Target direction - start direction. A part of rotational angle determination procedure.
		if (delta < 0)
			delta += 4;
		actionMoveTurn->turnByCCW = 90 * delta; // This is the rotational angle.
		print("Turn by %i CCW\n\r", (int)actionMoveTurn->turnByCCW); // For debugging.
		actionMoveTurn->endAngle = mrm_imu->heading() - actionMoveTurn->turnByCCW; // Now find the end heading (angle).
		// If the result is beyond 0 - 360 degrees limits, bring it again into the needed segment.
		if (actionMoveTurn->endAngle > 360)
			actionMoveTurn->endAngle -= 360;
		else if (actionMoveTurn->endAngle < 0)
			actionMoveTurn->endAngle += 360;
		//print("Start %i, end %i, ", (int)mrm_imu->heading(), (int)actionMoveTurn->endAngle);
		//print("TURN by %i CCW\n\r", (int)actionMoveTurn->turnByCCW);
		actionSet(actionMoveTurn); // The next action will be turning.
		actionMoveTurn->startMs = millis(); // Mark the start time.
	}
}

/** Drives the robot straight ahead.
*/
void RobotMaze::moveAhead() {
	bool encodersOver = false; // One tile's length covered. Used only when encoders available.
	// When encoders available, timeout is only a safety measure, to avoid possible endless loop. When not, it marks end of tile.
	bool timeOver = millis() - actionMoveAhead->startMs > MOVE_AHEAD_TIMEOUT_MS; 

	// If any of the 3 conditions satisfied, break the movement: encoder, timeout, or a wall to close ahead.
	if (encodersOver || timeOver || distance(directionCurrent, false) < WALL_FOLLOW_DISTANCE || distance(directionCurrent, true) < WALL_FOLLOW_DISTANCE) {
		// The robot's position changed so we need to calculate new coordinates.
		int8_t newX = tileCurrent->x;
		int8_t newY = tileCurrent->y;
		if (directionCurrent == UP)
			newY++;
		else if (directionCurrent == LEFT)
			newX--;
		else if (directionCurrent == DOWN)
			newY--;
		else
			newX++;
		print("Coord: (%i, %i) ", newX, newY); // Debug.

		Tile* targetTile = tileContaining(newX, newY); //Search chain for a tile already containing (x, y) - new robot's coordinates.
		if (targetTile != NULL) { // A tile found. That means we returned to the already visited tile.
			print("tile exists\n\r"); // Debug.
			tileCurrent = targetTile; // Set the position to the found tile.
			actionSet(actionDecide); // Movement over. The next action will be decision what to do next. No mapping is needed here as the tile has already been mapped.
		}
		else { // No such a tile. Therefore, this coordinate has not been visited yet. We have to create a new Tile object.
			print("new tile\n\r");
			// Set the position to a newly created tile, with new (x, y) coordinates. Breadcrumb direction will be the opposite direction to the current one.
			tileCurrent = new Tile(newX, newY, (Direction)((directionCurrent + 2) % 4)); 
			actionSet(actionMap); // The next action will be tile mapping as this is a new tile and its walls are not mapped yet.
		}
	}
	else { // End of tile not reached yet. Note that this part will execute many times during one-tile trip.
		Direction closestWall = wallClosest(); // Find the wall most appropriate to align with.
		if (closestWall == NOWHERE) // No alignable wall, the movement can be corrected only by compass (IMU).
			imuFollow();
		else // Yes, a wall found in one direction. Follow it.
			wallFollow(closestWall);
	}
}

/** Turns the robot till the target bearing achieved.
*/
void RobotMaze::moveTurn() {
	// Note that this function will execute many times during a single turn.
	int8_t speed = actionMoveTurn->turnByCCW > 180 ? TOP_SPEED : -TOP_SPEED; // Determine if it is better (smaller angle) to turn CW or CCW.
	motorGroup->go(speed, -speed); // Turn on the motors. In fact, only first run will do anything as the speeds will not change later.
	if (fabs(mrm_imu->heading() - actionMoveTurn->endAngle) < 5) { // If angle error is less than 5 degrees, action over.
		motorGroup->stop();
		directionCurrent = actionMove->direction; // Robot's direction changed after turn so store the new value.
		// Remember the new heading that IMU following algorithm uses. If no wall to alignment after turn, this value will be used immediately.
		imuLastValid = actionMoveTurn->endAngle; 
		actionSet(actionDecide); // After rotation, the tile stays the same, and no mapping will be needed. The new action is decision what to do next.
	}

	//print("T: %i - %i = %i\n\r", (int)mrm_imu->heading(), (int)actionMoveTurn->endAngle, (int)fabs(mrm_imu->heading() - actionMoveTurn->endAngle));

	/* A safety measure as timeout elapsed. Most probably, the robot is stuck. In this example program we will just quit. In a production program
	evasive actions should be performed, like turning CW-CCW vigorously, or going a little forward. We will have to do whatever possible to rotate
	the robot in the desired heading. Without success, the exit bonus will probably be lost in this point.*/
	if (millis() - actionMoveTurn->startMs > 5000) {
		motorGroup->stop();
		print("Turn timeout %i->%i\n\r", actionMoveTurn->startMs, millis());
		end(); // End of program.
	}
}


/** Test for Mecanum wheels. Used only when the robot is rigged with mecanum wheels. Not considered here.
*/
void RobotMaze::omniWheelsTest() {
	static uint8_t nextMove;
	static uint32_t lastMs;
	if (setup()) {
		if (motorGroup == NULL) {
			print("Differential motor group needed.");
			end();
			return;
		}
		nextMove = 0;
		lastMs = millis();
	}
	switch (nextMove) {
	case 0:
		if (millis() - lastMs > 2000) {
			lastMs = millis();
			nextMove = 1;
		}
		else
			motorGroup->go(20, 20, 70);
		break;
	case 1:
		if (millis() - lastMs > 15000) {
			lastMs = millis();
			nextMove = 2;
		}
		else {
			float angle = (millis() - lastMs) / 1500.0;
			int8_t x = cos(angle) * 50;
			int8_t y = sin(angle) * 50;
			motorGroup->go(y, y, x);
			//print("%i %i %i\n\r", (int)(angle*100), (int)(x * 100), (int)(y * 100));
			//delay(100);
		}
		break;
	case 2:
		if (millis() - lastMs > 2000) {
			lastMs = millis();
			nextMove = 0;
		}
		else
			motorGroup->go(-20, -20, -70);
		break;
	}
}

/** Starts RCJ Rescue Maze run.
*/
void RobotMaze::rescueMaze() {
	print("Maze start\n\r");
	// mrm-8x8a is positioned so that its bottom is aligned robot's left side. Rotate the display so that the image is aligned with robot's back.
	mrm_8x8a->rotationSet(LED_8X8_BY_90_DEGREES); 
	bitmapsSet(); // Upload custom bitmaps into mrm-8x8a.
	mrm_8x8a->bitmapCustomStoredDisplay(LedSign::MAZE_LED_PLAY); // Display play sign.
	Tile::first = new Tile(0, 0, NOWHERE); // Set the first tile and start the chain. Tile::first will point to its head, enabling iterating.
	tileCurrent = Tile::first; // Current tile is the first tile.
	/* This direction determines how the maze will look to a human user. For the robot it has no immportance. If it is UP, and we stand behind robot's
	back, the maze will look natural, as maze's top part will be at the top of displayed bitmaps, etc. If we used DOWN, and the same standpoint,
	the maze would be upside down.*/
	directionCurrent = UP;
	stepCount = 0; // Step counter. Used to stop a robot after a certain number of steps, if needed.
	actionSet(actionMap); // The robot is on uncharted tile so first action will be wall mapping.
	delayMs(200); // Debounce switch - the switch is turned on repeatedly due to its inductance. Skip that time in a crude manner - just wait.
}

/** Traverses all the chain till a tile with (x,y) is found.
@param x - x coordinate.
@param y - y coordinata.
@return - a tile with given coordinates and NULL if none exists.
*/
Tile* RobotMaze::tileContaining(int8_t x, int8_t y) {
	for (Tile* curr = Tile::first; curr != NULL; curr = curr->next()) // Use internal links to traverse the whole linked list.
		if (curr->x == x && curr->y == y) // Tile found.
			return curr; // Search over.
	return NULL;
}

/** Finds the closest of the 4 walls around the tile.
@return - direction from maze's perspective.
*/
Direction RobotMaze:: wallClosest() {
	Direction closest = NOWHERE; // If no wall find, this will be the result.
	uint16_t leastDistance = 0xFFFF; // Set variable to a huge number so that any distance reading will override it.
	for (uint8_t i = 0; i < 4; i++) { // Iterate all the 4 directions.
		Direction dir = (Direction)i; // Cast integer to Direction.
		uint16_t d0 = distance(dir, false); // Take measurements of the both sensors in a chosen direction dir.
		uint16_t d1 = distance(dir, true);
		//If both of the distances read walls and their difference is not too big, we conclude that there is a wall in this direction.
		if (d0 < NO_WALL_DISTANCE && d1 < NO_WALL_DISTANCE && abs(d0 - d1) < WALL_FOLLOW_ERROR_ALLOWED) {
			if (d0 < d1) // Find the smaller of 2 values.
				d0 = d1;
			if (d0 < leastDistance) { // If it small enought, change the running minimum.
				leastDistance = d0;
				closest = dir; // Remember the direction of the new minimum.
			}
		}
	}
	return closest;
}

/** Goes ahead by following a wall.
@wallDirection - direction in maze's perspective.
*/
void RobotMaze::wallFollow(Direction wallDirection) {
	// Use mrm-8x8a to display the wall to follow.
	if (wallDirection == Direction::UP)
		mrm_8x8a->bitmapCustomStoredDisplay(LedSign::MAZE_WALL_UP_FOLLOW);
	else if (wallDirection == Direction::DOWN)
		mrm_8x8a->bitmapCustomStoredDisplay(LedSign::MAZE_WALL_DOWN_FOLLOW);
	else if (wallDirection == Direction::LEFT)
		mrm_8x8a->bitmapCustomStoredDisplay(LedSign::MAZE_WALL_LEFT_FOLLOW);
	else
		mrm_8x8a->bitmapCustomStoredDisplay(LedSign::MAZE_WALL_RIGHT_FOLLOW);

	// Correct alignment to the wall (rotation around vertical axis).
	int16_t differenceCWMinusCCW = distance(wallDirection, false) - distance(wallDirection, true); // Rotational error.
	// If differenceCWMinusCCW positive, the robot is rotated too much CW and positive speedR will decrease the error.
	int16_t speedR = differenceCWMinusCCW * WALL_FOLLOW_ROTATION_STRENGTH; // speedR will hold cumulative corrections. In the end we will calculate speedL.

	// Correct distance to the wall. The robot should maintain WALL_FOLLOW_DISTANCE distance.
	int16_t closerToWallThanOptimum = WALL_FOLLOW_DISTANCE - (distance(wallDirection, false) + distance(wallDirection, true)) / 2; // Error. Negative - too far
	int16_t distanceAdjustmentCW;
	// Correction of alignment error was easier as it did not depend on direction, but this one does, so we have to consider different directions.
	if (directionCurrent == wallDirection || (directionCurrent + 2) % 4 == wallDirection)  // Follow wall ahead or behind the robot.
		distanceAdjustmentCW = 0; // This direction cannot be used, but this part of the algorithm can be improved, too! 0 is not the best option.
	else if ((directionCurrent + 1) % 4 == wallDirection) // Follow wall to the left of the robot.
		distanceAdjustmentCW = closerToWallThanOptimum * WALL_FOLLOW_DISTANCE_ADJUSTMENT_STRENGTH;
	else // Follow wall to the right
		distanceAdjustmentCW = -closerToWallThanOptimum * WALL_FOLLOW_DISTANCE_ADJUSTMENT_STRENGTH;
	speedR -= distanceAdjustmentCW; // Cumulative change of speedR.

	int16_t toTheMax = TOP_SPEED - abs(speedR); // How much more to get maximum speed?
	int16_t speedL = -speedR + toTheMax; // Increase L and R speeds so that one of them hits TOP_SPEED.
	speedR += toTheMax;

	//directionDisplay(wallDirection);
	//print(" Wall follow CW:%i, CCW:%i, L: %i, R:%i\n\r", distance(wallDirection, false), distance(wallDirection, true), speedL, speedR);
	motorGroup->go(speedL, speedR); // Engage motors.

	// Correct stored direction. It is constantly adjusted when following a wall. It is mandatory because magnetic field can change much, even in one tile.
	imuLastValid = mrm_imu->heading(); 
}

/** Checks if wall exists in tile (x,y) and direction direction.
@param x - tile's x coordinate.
@param y - tile's y coordinate.
@param direction - wall's direction.
@return - type of wall.
*/
WallStatus RobotMaze::wallGet(int8_t x, int8_t y, Direction direction) {
	for (Tile* tile = Tile::first; tile != NULL; tile = tile->next()) // Iterate all the tiles.
		if (tile->x == x && tile->y == y && tile->wallGet(direction) != WallStatus::WALL_UNKNOWN) // Correct (x, y) and wall status known - return it.
			return tile->wallGet(direction);
	return WallStatus::WALL_UNKNOWN;
}

/** Display one wall.
@param wallStatus - wall's status.
@param direction - direction.
*/
void RobotMaze::wallDisplay(WallStatus wallStatus, Direction direction) {
	if (direction > Direction::NOWHERE) // Argument checking.
		strcpy(errorMessage, "Dir. err.");
	switch (wallStatus)
	{
	case NO_WALL:
		print(" ");
		break;
	case WALL_UNKNOWN:
		print("?");
		break;
	case WALL_WITH_VICTIM:
		print("☻"); // Unicode sign. Terminal must be UTF-8 friendly in order to display this sign.
		break;
	case WALL_WITHOUT_VICTIM:
		if (direction == Direction::UP || direction == Direction::DOWN)
			print("─"); // UTF-8.
		else
			print("│"); // UTF-8.
		break;
	default:
		print("?");
		break;
	}
}

/** Use 8x8 LED to display walls detected.
*/
void RobotMaze::wallsDisplay() {
	/* Contrary to the method used mostly here, this bitmap is not stored beforehand. There are too many combinations and that will not be feasible.
	So, it will be built on-line.*/
	// Clear red and green dots.
	uint8_t red[8];
	uint8_t green[8];
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0, green[i] = 0;

	//2 top sensors form top pixels' row.
	if (distance(rToM(Direction::UP), false) < NO_WALL_DISTANCE)
		red[0] |= 0b00001111;
	if (distance(rToM(Direction::UP), true) < NO_WALL_DISTANCE)
		red[0] |= 0b11110000;

	//2 left sensors form left row.
	if (distance(rToM(Direction::LEFT), false) < NO_WALL_DISTANCE)
		for (uint8_t i = 0; i < 4; i++)
			red[i] |= 0b10000000;
	if (distance(rToM(Direction::LEFT), true) < NO_WALL_DISTANCE)
		for (uint8_t i = 4; i < 8; i++)
			red[i] |= 0b10000000;

	// 2 bottom sensors.
	if (distance(rToM(Direction::DOWN), false) < NO_WALL_DISTANCE)
		red[7] |= 0b11110000;
	if (distance(rToM(Direction::DOWN), true) < NO_WALL_DISTANCE)
		red[7] |= 0b00001111;

	// Finally, 2 right sensors.
	if (distance(rToM(Direction::RIGHT), false) < NO_WALL_DISTANCE)
		for (uint8_t i = 4; i < 8; i++)
			red[i] |= 0b00000001;
	if (distance(rToM(Direction::RIGHT), true) < NO_WALL_DISTANCE)
		for (uint8_t i = 0; i < 4; i++)
			red[i] |= 0b00000001;

	// Send the result to mrm-8x8a.
	mrm_8x8a->bitmapCustomDisplay(red, green);
}

/** Test, checking and displaying walls.
*/
void RobotMaze::wallsTest() {
	if (setup()) { // First run of the action.
		directionCurrent = Direction::UP; // We should be standing behind the robot. Otherwise, change this value.
		mrm_8x8a->rotationSet(LED_8X8_BY_90_DEGREES); // Rotate the display as it is mounted rotated by 90 degrees.
	}
	// Display the values in terminals.
	print(" FR:%i", distance(Direction::UP, false));
	print(" FL:%i", distance(Direction::UP, true));
	print(" LF:%i", distance(Direction::LEFT, false));
	print(" LB:%i", distance(Direction::LEFT, true));
	print(" BL:%i", distance(Direction::DOWN, false));
	print(" BR:%i", distance(Direction::DOWN, true));
	print(" RB:%i", distance(Direction::RIGHT, false));
	print(" RF:%i\n\r", distance(Direction::RIGHT, true));

	wallsDisplay(); // Display using mrm-8x8a.

	delayMs(200);
}

/** x coordinate of the tile next to the current one, in a given direction.
@param direction.
@return x coordinate.
*/
int8_t RobotMaze::x(Direction direction) { 
	// A trivial calculation.
	if (direction == Direction::LEFT)
		return tileCurrent->x - 1;
	else if (direction == Direction::RIGHT)
		return tileCurrent->x + 1;
	else
		return tileCurrent->x;
}

/** y coordinate of the tile next to the current one, in a given direction.
@param direction.
@return y coordinate.
*/
int8_t RobotMaze::y(Direction direction) {
	// A trivial calculation.
	if (direction == Direction::UP)
		return tileCurrent->y + 1;
	else if (direction == Direction::DOWN)
		return tileCurrent->y - 1;
	else
		return tileCurrent->y;
}

/** Constructor.
@param x - x coordinate.
@param y - y coordinate.
@param breadcrumb - way back to the start tile.
*/
Tile::Tile(int8_t xNow, int8_t yNow, Direction breadcrumbNow) {
	x = xNow;
	y = yNow;
	breadcrumb = breadcrumbNow;
	if (breadcrumb != NOWHERE) { // Otherwise it is the first tile and no chain from the previous tile is needed.
		// Iterate all the tiles in order to find the last one.
		Tile* tile = NULL;
		for (tile = first; tile->_chain != NULL; tile = tile->_chain)
			;
		if (tile != NULL)
			tile->_chain = this; // Set last tile's link to this, newly created, tile.
	}
	_chain = NULL; // As this is the new end of chain, there is no tile it points to.
	_wall = 0b01010101; // Unknown walls, stored in bits.
}
