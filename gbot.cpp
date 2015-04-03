// #include <Adafruit_TiCoServo.h>
#include "gbot.h"
#include "parameters.h"

namespace robot {

byte lift_pin;
byte ball_pin;
bool ball_dropped;
byte ball_status = BALL_LESS;


byte slots[GAME_COLS][GAME_HEIGHT];
Slot top_slots[GAME_COLS];	
// col 1, col 2, ... indices return their actual physical locations
Target col_pos[GAME_COLS+1] = {{RENDEZVOUS_X, RENDEZVOUS_Y, HALFPI, TARGET_WATCH}, {RENDEZVOUS_X, 665, HALFPI, TARGET_PLAY}, 
	{RENDEZVOUS_X, 710, HALFPI, TARGET_PLAY}, {RENDEZVOUS_X, 755, HALFPI, TARGET_PLAY}, {RENDEZVOUS_X, 800, HALFPI, TARGET_PLAY}, 
	{RENDEZVOUS_X, 845, HALFPI, TARGET_PLAY}, {RENDEZVOUS_X, 890, HALFPI, TARGET_PLAY}, {RENDEZVOUS_X, 935, HALFPI, TARGET_PLAY}};

byte best_top_slot;
byte continuous_ours = 0;
byte continuous_theirs = 0;
bool streak = true;
byte prev_slot = OUR_BALL;
int score = 0;

int rel_pos;

// sensor bar
byte bar_num;	// number of bar sensors
byte lasers[BAR_MAX];
byte bars[BAR_MAX];
int ambient[BAR_MAX];
byte ball_drops, prev_ball_drops;
byte consecutive_drops;
bool played_ball;
bool jammed;
unsigned long last_dropped_ours, last_dropped_theirs;
unsigned long last_calibrate_time;

// correction (only position)
int cycles_on_line;
int counted_lines;
byte board_status;
byte corrected_y;


byte trigs[SONAR_MAX];
byte echos[SONAR_MAX];
float prev_wall_distance[SONAR_MAX];
float wall_distance[SONAR_MAX];
byte sonar_num;
byte sonar_cycle;

bool ready_to_hug_wall() {
	if (active_layer == LAYER_PLAY) return true;
	else if (board_status == STARTING_LOCATION) return false;
	// only hug wall when close to a wall
	int approx_heading = square_heading();
	// sonar can't correct beyond a certain angle
	if (abs(approx_heading - theta*RADS) > SONAR_THETA_MAXIMUM) return false; 
	// heading left, x should be close to 0
	else if ((approx_heading == DIR_LEFT && x < SONAR_CLOSE_ENOUGH) ||
	// heading up, y should be close to 0
	(approx_heading == DIR_UP && y < SONAR_CLOSE_ENOUGH) ||
	// heading right, x should be close to GAME_BOARD_X
	(approx_heading == DIR_RIGHT && GAME_BOARD_X - x < SONAR_CLOSE_ENOUGH)) return true;
	return false; 
}

// called inside every go cycle
void user_behaviours() {
	play_ball();
	// reset wall distances
	if (active_layer == LAYER_TURN && prev_wall_distance[0] != 0)
		for (byte sonar = 0; sonar < sonar_num; ++sonar) prev_wall_distance[sonar] = 0;
	// first reaching the left wall
	if (board_status == STARTING_LOCATION && y - 0 < SONAR_CLOSE_ENOUGH) {
		board_status = LEFT_WALL;
		SERIAL_PRINTLN("BL");
	}
	else if (board_status == LEFT_WALL && GAME_BOARD_X - x < SONAR_CLOSE_ENOUGH) {
		// calibrate y position before turning
		if (corrected_y >= RELIABLE_CORRECT_CYCLE) {
			board_status = GAME_WALL;
			add_target(x, y, 90, TARGET_TURN);
			SERIAL_PRINTLN("BG");
		}
	}
	// don't turn in place when you're too close to the board
	else if (board_status == GAME_WALL && HALFPI - theta < THETA_TOLERANCE) {
		bitClear(allowed_layers, LAYER_TURN);
		layers[LAYER_TURN].active = false;
		board_status = GOING_TO_PLAY;
		// go to rendezvous location
		add_target(RENDEZVOUS_X, RENDEZVOUS_Y, 90, TARGET_WATCH);
		SERIAL_PRINTLN("BGP");
	}
	// stop navigating when rendezvous reached the first time
	else if (board_status == GOING_TO_PLAY && RENDEZVOUS_Y - y < RENDEZVOUS_CLOSE) {
		bitClear(allowed_layers, LAYER_NAV);
		layers[LAYER_NAV].active = false;
		layers[LAYER_WATCH].active = true;
		board_status = PLAYING;
		// start watching
		SERIAL_PRINTLN("BP");
	}


	if (ready_to_hug_wall()) {
		if (sonar_cycle == WALL_DISTANCE_READY) {
			hug_wall();
			sonar_cycle = 0;
		}
	}
	else {
		// reset cycles after hugging wall
		if (sonar_cycle == WALL_DISTANCE_READY) sonar_cycle = 0;
	}
}

// control the correction layer
void user_correct() {
	// parallel_park();
	if (layers[LAYER_TURN].active) return;
	watch_balls_drop();
	passive_position_correct();
	if (ready_to_hug_wall()) touch_wall();
}

// called after arriving
void user_waypoint() {
	// listen to game whenever not moving
	if (board_status == PLAYING) {
		// assume rests at 4th pillar (rendezvous)
		rel_pos = COL_4;
		hard_break();
		layers[LAYER_WATCH].active = true;
	}
}


void initialize_gbot(byte lift, byte ball) {
	ball_dropped = false;
	// PWM to control ball lift
	lift_pin = lift;
	pinMode(lift_pin, OUTPUT);
	ball_pin = ball;
	pinMode(ball_pin, INPUT);

	continuous_ours = 0;
	continuous_theirs = 0;
	best_top_slot = 3;
	streak = true;
	prev_slot = OUR_BALL;

	rel_pos = AWAY_FROM_BOARD;

	bar_num = 0;	// number of bar sensors
	ball_drops = BALL_LESS;
	consecutive_drops = 0;
	played_ball = false;
	jammed = false;
	last_dropped_ours = last_dropped_theirs = 0;
	last_calibrate_time = 0;

	cycles_on_line = 0;
	counted_lines = 0;
	board_status = STARTING_LOCATION;
	corrected_y = 0;

	sonar_num = 0;
	sonar_cycle = 0;

	// initialize game board
	for (byte col = 0; col < GAME_COLS; ++col) {
		for (byte row = 0; row < GAME_HEIGHT; ++row) {
			slots[col][row] = NO_BALL;
		}
	}
	for (byte col = 0; col < GAME_COLS; ++col) {
		top_slots[col] = {col, 0, 0};	// initially 0 height and 0 score
	}

	layers[LAYER_WATCH].speed = 0;
	layers[LAYER_WATCH].angle = 0;
}

void user_start() {
	sonar_cycle = 0;
}


}	// end namespace