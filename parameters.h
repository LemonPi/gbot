#pragma once
#include <Arduino.h>

#define DEBUGGING

#ifdef DEBUGGING
	#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__);
	#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__);
#else
	#define SERIAL_PRINT(...) ;
	#define SERIAL_PRINTLN(...) ;
#endif

namespace robot {
// mathematical constants
constexpr float TWOPI = 6.2831853070;
constexpr float HALFPI = 1.5707963268;
constexpr float QUARTERPI = 0.78539816339;
constexpr float DEGS = 0.0174532925;
constexpr float RADS = 57.29577951;

// drive modes
constexpr bool MANUAL = false;
constexpr bool AUTOMATIC = true;

// subsumption layers
constexpr byte LAYER_NUM = 7;
constexpr byte LAYER_BOUND = 0; // highest priority
constexpr byte LAYER_PLAY = 1;
constexpr byte LAYER_COR = 2;
constexpr byte LAYER_TURN = 3;
constexpr byte LAYER_NAV = 4;
constexpr byte LAYER_WATCH = 5;
constexpr byte LAYER_WAIT = 6;
constexpr byte LAYER_GET = 7;	// non-existent layer
constexpr byte ALL_ALLOWED = B11111111;

constexpr int CYCLE_TIME = 20; 	// in ms
constexpr int SENSOR_TIME = 10;  // in ms, 5x faster than navigation cycles

// maximum array bounds
constexpr byte BOUNDARY_MAX = 0;
constexpr byte TARGET_MAX = 10;
constexpr byte SENSOR_MAX = 1;
constexpr byte SONAR_MAX = 2;

// target types
constexpr byte TARGET_NAV = 0;
constexpr byte TARGET_TURN = 1;
constexpr byte TARGET_PLAY = 2;
constexpr byte TARGET_WATCH = 3;
constexpr byte TARGET_GET = 4;

// sensor indices and general direction
constexpr byte CENTER = B0001;		// center sensor is sensor 0
constexpr byte RIGHT = 	B0010;
constexpr byte LEFT = 	B0100;
constexpr byte UP = 	B1000;
constexpr float SIDE_SENSOR_DISTANCE = 43.5;


// PID speed control
constexpr int FORWARD = 1;
constexpr int BACKWARD = -1;
constexpr char START_DRIFT = 1;	// how many cycles to account for drifting by using previous direction
constexpr char NO_DRIFT = 0;
constexpr int DRIFT_SPEED = 15;	// how fast of a speed change to start accounting for drift


constexpr float BASE_WIDTH = 99.0;
constexpr float TURNING_RADIUS = BASE_WIDTH;
constexpr float RECIPROCAL_BASE_WIDTH = 0.01004009;	// using reciprocal due to faster multiply than divide
constexpr float MM_PER_TICK_L = 0.1688929559*1000/1045;
constexpr float MM_PER_TICK_R = 0.16956084148*1000/1045;

constexpr float KP = 1.194;
constexpr float KI = 1.2;
constexpr float KD = 0.005;

constexpr int TPR = 1200;

constexpr int TOP_SPEED = 50; 	// in ticks per cycle 
constexpr int MIN_SPEED = 20;
constexpr int START_SPEED = 45;


// navigation
constexpr float TARGET_CIRCLE = 10.0;	// allow for 20mm error from target
constexpr float TARGET_IMMEDIATE = 5.0;// don't try to get closer than 5mm (fixes high heading error when really close)
constexpr float TARGET_CLOSE = 200.0;	// slow down 100mm from target
constexpr int NAV_TURN = 20;			// turn size in ticks/cycle, adjustable
constexpr float THETA_TOLERANCE = 0.03;	// around 6 degree turning

constexpr int ANY_THETA = 9000;	// if no target is set
constexpr float CAN_TURN_IN_PLACE = 0.5; // minimum angle to activate turning in place



// boundary avoidance
constexpr int BOUNDARY_TOO_CLOSE = 200;	// 200 mm from center of wheels
constexpr int BOUNDARY_FAR_ENOUGH = 100;	// can stop tracking active boundary this far away
constexpr float BOUNDARY_TOLERANCE = HALFPI; // corresponds to how wide it is
constexpr float EXISTENTIAL_THREAT = 0.3*BOUNDARY_TOO_CLOSE;

constexpr int BOUND_TURN = 20;		// how hard to turn away from obstacle; adjustable

constexpr int NONE_ACTIVE = -1;



// line detecting sensors
constexpr int CALLIBRATION_TIME = 5000;	// 5s
constexpr int THRESHOLD_TOLERANCE = 3;
constexpr int LINE_WIDTH = 9; 			// about 1cm
constexpr float HALF_LINE_WIDTH = 4.5;
constexpr int GRID_WIDTH = 200;			// grid spaced about 200mm apart
constexpr int CYCLES_CROSSING_LINE = 2; 	// cycles on line for false positive to fail
constexpr int LINES_PER_CORRECT = 0;	// how many lines to cross before correcting; 0 is every line

// correct to line directions
constexpr int DIR_UP = 0;
constexpr int DIR_LEFT = -90;
constexpr int DIR_RIGHT = 90;
constexpr int DIR_BACK = 180;


// correction
constexpr byte INTERSECTION_TOO_CLOSE = 40;	// allowed range [50,150] for x and y for a correct

// sonar correction
constexpr float START_PARALLEL_PARK = 0.3;	// start using sonar instead of turning in place at around 17 degrees
constexpr int COR_TURN = 10; 
constexpr float DISTANCE_FROM_PULSE = 0.34364261168;
constexpr byte SONAR_CYCLE = 5;
constexpr byte WALL_DISTANCE_READY = SONAR_CYCLE + 1;	// signifies wall distances are calculated
constexpr int GAME_BOARD_X = 1800;
constexpr byte RELIABLE_CORRECT_CYCLE = 2;	// how many correction cycles before considered stable value 
constexpr int RELIABLE_SONAR_DISTANCE = 45;

constexpr byte SIDE_FRONT = 0;
constexpr byte SIDE_BACK = 1;
constexpr byte FRONT = 2;
constexpr float WALL_DISTANCE = 100;		// ideally 5cm away
constexpr float SONAR_DISTANCE_TOLERANCE = 3.5;
constexpr float SONAR_CHANGE_ALLOWANCE = 100;	// allow a difference of 40mm from current to previous reading
constexpr float SONAR_TOO_FAR = 2000;		// from sonar to wall
constexpr float SONAR_CLOSE_ENOUGH = 200;	// from center of the robot to the wall
constexpr int SONAR_THETA_MAXIMUM = 25;	// cannot reliably correct beyond 25 degrees

// board statuses
constexpr byte STARTING_LOCATION = 0;
constexpr byte LEFT_WALL = 1;
constexpr byte GAME_WALL = 2;
constexpr byte GOING_TO_PLAY = 3;
constexpr byte PLAYING = 4;


// aligning to wall
constexpr int SIDE_FRONT_OFFSET = 2;
constexpr float CENTER_TO_SONAR_DISTANCE = 17;
constexpr int SIDE_SONAR_DISTANCE = 111+123;
constexpr float SIDE_FRONT_BACK_RATIO = 123/SIDE_SONAR_DISTANCE;


// playing the ball
constexpr int PLAY_SPEED = 20;	// how fast to move between game board column locations
constexpr int RENDEZVOUS_X = GAME_BOARD_X - WALL_DISTANCE - CENTER_TO_SONAR_DISTANCE + 10;
constexpr int RENDEZVOUS_Y = 890;
constexpr int RENDEZVOUS_CLOSE = 5;	// within 5mm of rendezvous
constexpr byte GAME_COLS = 7;
constexpr byte GAME_HEIGHT = 6;	// 6 balls max
constexpr byte SCORE_DEPTH = 3;	// only consider 3 away from the ball
constexpr byte TWO_IN_ROW = 2;
constexpr byte THREE_IN_ROW = 3;
constexpr int LIFT_SPEED = 130;


constexpr byte NO_BALL  = 0;
constexpr byte OUR_BALL = 1;
constexpr byte THEIR_BALL = 2;
constexpr unsigned long DROPPED_TOO_RECENTLY = 10000;	// assume no one plays balls within 10s

// relative positions
constexpr byte COL_1 = 0;
constexpr byte COL_2 = 1;
constexpr byte COL_3 = 2;
constexpr byte COL_4 = 3;
constexpr byte COL_5 = 4;
constexpr byte COL_6 = 5;
constexpr byte COL_7 = 6;
constexpr byte AWAY_FROM_BOARD = 7;
constexpr byte COL_CLOSE = 10;
constexpr byte COL_WIDTH = 45;

// ball statuses
constexpr byte BALL_LESS = 0;
constexpr byte JUST_GOT_BALL = 1;
constexpr byte SECURED_BALL = 5;	// cycles of gate closing
constexpr byte BALL_TO_BE_DROPPED = 350;	// 3s for it to rise

// sensor bar
constexpr byte BAR_MAX = 7;
constexpr int AMBIENT_THRESHOLD = 30;	// offset threshold to be considered not ambient
constexpr byte REAL_DROP = 3;	// consecutive cycles of detecting a ball drop 

}