// #include <Adafruit_TiCoServo.h>
#include "parameters.h"
#include "robot.h"

namespace robot {

struct Slot {
	byte c, h;
	int score;
};

// lifting the ball
extern byte lift_pin;
extern byte ball_pin;
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
extern int rel_pos;
extern int score;


// sensor bar
extern byte bar_num;	// number of bar sensors
extern byte laser_pin;
extern byte lasers[BAR_MAX];
extern byte bars[BAR_MAX];
extern int ambient[BAR_MAX];
extern byte ball_drops, prev_ball_drops;
extern byte consecutive_drops;
extern bool played_ball;
extern bool jammed;
extern unsigned long last_dropped_ours, last_dropped_theirs;
extern unsigned long last_calibrate_time;

// correction
extern int cycles_on_line;
extern int counted_lines;
extern byte board_status;
extern byte corrected_y;
extern byte corrected_x;

// sonar
extern byte trigs[SONAR_MAX];
extern byte echos[SONAR_MAX];
extern float prev_wall_distance[SONAR_MAX];
extern float wall_distance[SONAR_MAX];
extern const float wall_distance_offset[SONAR_MAX];
extern byte sonar_num;
extern byte sonar_cycle;
extern byte turned_to_watch;
extern float turn_factor;

extern unsigned long last_random_lift;
extern unsigned long last_lift;


void initialize_gbot(byte lift_p, byte ball_p, byte laser_p);


// play module (deposit ball)
void play_ball();
void lift_ball();
void lift_ball_harder();
void stop_lift_ball();
int score_top_slot(byte top);
void reset_run();
void score_slot(byte c, byte h);
void pick_best_col();

// watch module (listen to sensor bar)
void turn_to_watch();
void watch_balls_drop();
void fire_lasers(int level);
void watch_bar();
int add_bar_sensor(byte sensor_pin, byte laser_pin);
void calibrate_bar(unsigned long duration = CALLIBRATION_TIME);
bool received_ball();
void drop_off_ball();
void update_game_board();

// user correct
byte add_sonar(byte t, byte e);
void passive_position_correct();
void parallel_park();
void touch_wall();
void hug_wall();
bool ready_to_hug_wall();
void reset_wall_distance();
bool far_from_intersection_gbot(int xx, int yy);

}