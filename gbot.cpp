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
Target col_pos[GAME_COLS+1] = {{RENDEZVOUS_X, RENDEZVOUS_Y-3*COL_WIDTH, HALFPI, TARGET_PLAY}, 
	{RENDEZVOUS_X, RENDEZVOUS_Y-2*COL_WIDTH, HALFPI, TARGET_PLAY}, {RENDEZVOUS_X, RENDEZVOUS_Y-1*COL_WIDTH, HALFPI, TARGET_PLAY}, {RENDEZVOUS_X, RENDEZVOUS_Y, HALFPI, TARGET_PLAY}, 
	{RENDEZVOUS_X, RENDEZVOUS_Y+1*COL_WIDTH, HALFPI, TARGET_PLAY}, {RENDEZVOUS_X, RENDEZVOUS_Y+2*COL_WIDTH, HALFPI, TARGET_PLAY}, {RENDEZVOUS_X, RENDEZVOUS_Y+3*COL_WIDTH, HALFPI, TARGET_PLAY},
	{RENDEZVOUS_X, RENDEZVOUS_Y, HALFPI, TARGET_WATCH}};

byte best_top_slot;
byte continuous_ours = 0;
byte continuous_theirs = 0;
bool streak = true;
byte prev_slot = OUR_BALL;
int score = 0;

int rel_pos;

// sensor bar
byte bar_num;	// number of bar sensors
byte laser_pin;
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
byte corrected_x;


byte trigs[SONAR_MAX];
byte echos[SONAR_MAX];
float prev_wall_distance[SONAR_MAX];
float wall_distance[SONAR_MAX];
const float wall_distance_offset[SONAR_MAX] = {-5,0};
byte sonar_num;
byte sonar_cycle;
byte turned_to_watch;
float turn_factor;



// called inside every go cycle
void user_behaviours() {

	play_ball();
	// reset wall distances
	if (active_layer == LAYER_TURN && prev_wall_distance[0] != 0)
		reset_wall_distance();
	// first reaching the left wall

	if (board_status == STARTING_LOCATION && y - 0 < SONAR_CLOSE_ENOUGH && square_heading() == DIR_UP && x > 600) {
		board_status = LEFT_WALL;
		SERIAL_PRINTLN("BL");
		disable_layer(LAYER_TURN);
		enable_layer(LAYER_COR);
	}
	// at top left corner, rotating to 90
	else if (board_status == LEFT_WALL && corrected_y >= RELIABLE_CORRECT_CYCLE && paused) {
		board_status = GAME_WALL;
		enable_layer(LAYER_TURN);
		add_target(x, y, 90, TARGET_TURN);
		resume_drive(LAYER_COR);
		SERIAL_PRINTLN("BG");
		disable_layer(LAYER_COR);
		reset_wall_distance();
	}
	// from top left corner to rendezvous
	// don't turn in place when you're too close to the board
	else if (board_status == GAME_WALL && corrected_x >= RELIABLE_CORRECT_CYCLE) {
		board_status = GOING_TO_PLAY;
		// go to rendezvous location
		disable_layer(LAYER_TURN);
		resume_drive(LAYER_NAV);
		add_target(RENDEZVOUS_X, RENDEZVOUS_Y, 90, TARGET_WATCH);
		// disable_layer(LAYER_COR);
		SERIAL_PRINTLN("BGP");
	}
	// stop navigating when rendezvous reached the first time
	else if (board_status == GOING_TO_PLAY && RENDEZVOUS_Y - y < RENDEZVOUS_CLOSE) {
		disable_layer(LAYER_TURN);
		disable_layer(LAYER_NAV);
		enable_layer(LAYER_COR);
		enable_layer(LAYER_WATCH);
		hard_break(LAYER_WATCH);
		layers[LAYER_WATCH].active = true;
		board_status = PLAYING;
		// start watching
		SERIAL_PRINTLN("BP");
	}


	if (allowed_layer(LAYER_COR)) {
		if (allowed_layer(LAYER_WATCH)) turn_to_watch();
		if (sonar_cycle == WALL_DISTANCE_READY) {
			hug_wall();
			sonar_cycle = 0;
		}
	}
	// if (active_layer == LAYER_PLAY) {
		// SERIAL_PRINT(heading_error * RADS);
		// SERIAL_PRINT('|');
		// SERIAL_PRINT(layers[active_layer].speed);
		// SERIAL_PRINT('|');
		// SERIAL_PRINTLN(layers[active_layer].angle);
	// }
}

// control the correction layer
void user_correct() {
	// parallel_park();
	if (layers[LAYER_TURN].active) return;
	watch_balls_drop();
	passive_position_correct();
	if (allowed_layer(LAYER_COR)) touch_wall();
}

// called after arriving
void user_waypoint() {
	turned_to_watch = 0;
	stop_lift_ball();

	if (board_status == LEFT_WALL && GAME_BOARD_X - x < SONAR_CLOSE_ENOUGH) {
		// calibrate y position before turning to BG
		SERIAL_PRINTLN("FR");
		enable_layer(LAYER_COR);
		disable_layer(LAYER_TURN);
		if (!paused) hard_break(LAYER_COR);
		// reset to force stationary correct cycles
		corrected_y = 0;
	}	
	else if (board_status == GAME_WALL) {
		hard_break(LAYER_TURN);
		SERIAL_PRINTLN("RW");
		reset_wall_distance();
		delay(500);
		enable_layer(LAYER_COR);
	}

	// nothing else to do, go back to rendezvous
	else if (target == NONE_ACTIVE) {
		add_target(RENDEZVOUS_X, RENDEZVOUS_Y, 90, TARGET_WATCH);
	}

	// arrived at rendezvous
	if (targets[target+1].type == TARGET_WATCH) {
		if (target == NONE_ACTIVE) target = 0;
		enable_layer(LAYER_WATCH);	
		// enable_layer(LAYER_COR);	
		layers[LAYER_PLAY].active = false;
		layers[LAYER_WATCH].active = true;
		turned_to_watch = 0;
		rel_pos = COL_4;	
		hard_break(LAYER_WATCH);
		SERIAL_PRINTLN("ren");
	}
	// arrived at a column
	else if (targets[target+1].type == TARGET_PLAY) {
		enable_layer(LAYER_WATCH);	
		// enable_layer(LAYER_COR);	
		layers[LAYER_PLAY].active = true;
		layers[LAYER_PLAY].speed = 0;
		layers[LAYER_PLAY].angle = 0;
		layers[LAYER_WATCH].active = true;
		turned_to_watch = 0;
		played_ball = false;
		rel_pos = best_top_slot;
		hard_break(LAYER_PLAY);
		SERIAL_PRINTLN("col");
	}

	// turn_to_watch();
}


void initialize_gbot(byte lift, byte ball, byte laser) {
	ball_dropped = false;
	// PWM to control ball lift
	lift_pin = lift;
	pinMode(lift_pin, OUTPUT);
	ball_pin = ball;
	pinMode(ball_pin, INPUT);
	laser_pin = laser;
	pinMode(laser_pin, OUTPUT);

	continuous_ours = 0;
	continuous_theirs = 0;
	best_top_slot = COL_4;
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
	corrected_x = 0;

	sonar_num = 0;
	sonar_cycle = 0;
	turned_to_watch = 0;
	turn_factor = 1;

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
	// only wall correct at certain points
	disable_layer(LAYER_COR);
	disable_layer(LAYER_WATCH);
}

void user_start() {
	sonar_cycle = 0;
	turned_to_watch = 0;
	turn_factor = 1;
}
void user_stop() {
	stop_lift_ball();
	fire_lasers(LOW);
}


}	// end namespace