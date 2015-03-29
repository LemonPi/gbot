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

byte rel_pos;

// sensor bar
byte bar_num;	// number of bar sensors
byte lasers[BAR_MAX];
byte bars[BAR_MAX];
int ambient[BAR_MAX];
byte ball_drops;
bool played_ball;

// correction (only position)
int cycles_on_line;
int counted_lines;

byte trigs[SONAR_MAX];
byte echos[SONAR_MAX];
float prev_wall_distance[SONAR_MAX];
float wall_distance[SONAR_MAX];
byte sonar_num;
byte sonar_cycle;

// called inside every go cycle
void user_behaviours() {
	play_ball();
	// if (layers[LAYER_TURN].active && !layers[LAYER_COR].active && abs(to_turn) < START_PARALLEL_PARK) {
	// 	SERIAL_PRINTLN(to_turn);
	// 	layers[LAYER_COR].active = true;
	// }
}

// control the correction layer
void user_correct() {
	// parallel_park();
	if (layers[LAYER_TURN].active) return;
	watch_balls_drop();
	passive_position_correct();
	hug_wall();
}

// called after arriving
void user_waypoint() {
	// listen to game whenever not moving
	if (target == NONE_ACTIVE) {
		stop();
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
	played_ball = false;

	cycles_on_line = 0;
	counted_lines = 0;

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