// #include <Adafruit_TiCoServo.h>
#include "parameters.h"
#include "robot.h"

namespace robot {

struct Slot {
	byte c, h, score;
};

// lifting the ball
extern byte lift_pin;
extern byte ball_pin;
extern bool ball_dropped;
extern byte ball_status;


// gameplay stratey
extern byte slots[GAME_COLS][GAME_HEIGHT];	// slots for balls (NO_BALL, OUR_BALL, THEIR_BALL)
extern Slot top_slots[GAME_COLS];			// top of the stack of balls
extern Target col_pos[GAME_COLS+1];			// physical location of the locations to play the ball

extern byte best_top_slot;
extern byte continuous_ours;
extern byte continuous_theirs;
extern bool streak;
extern byte prev_slot;
extern byte rel_pos;
extern int score;


// sensor bar
extern byte bar_num;	// number of bar sensors
extern byte lasers[BAR_MAX];
extern byte bars[BAR_MAX];
extern int ambient[BAR_MAX];
extern byte ball_drops;
extern bool played_ball;


extern float last_correct_distance;


void initialize_gbot(byte lift_p, byte ball_p);


// play module (deposit ball)
void play_ball();
void lift_ball();
void stop_lift_ball();
int score_top_slot(byte top);
void reset_run();
void score_slot(byte c, byte h);
void pick_best_col();

// watch module (listen to sensor bar)
void watch_balls_drop();
void fire_lasers();
void watch_bar();
int add_bar_sensor(byte sensor_pin, byte laser_pin);
void calibrate_bar();
bool received_ball();

// user correct
// correct theta at lines

}