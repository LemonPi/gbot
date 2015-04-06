#pragma once
#include <Arduino.h>
#include <hbridge.h>
#include "parameters.h"

namespace robot {

// each subsumption layer has a command, argument, and active
struct Layer {
	// speed and angle in units of ticks/cycle
	int speed, angle, active;
};
// each boundary has x, y, radius, distance away, heading error (theta), and threat
struct Boundary {
	double x, y, r, distance, theta, threat;
};
// each target has x and y coordinates in addition to optional theta
struct Target {
	double x, y, theta;
	byte type;
};

// 2 wheel differentially driven robot
// left input is connected to pin 2, right input is connected to pin 3
extern volatile int tick_l, tick_r;		// time between ticks
extern int instant_tick_l, instant_tick_r;
// subsumption layers
extern Layer layers[LAYER_NUM];
extern byte active_layer;
extern byte allowed_layers;

// avoid boundaries (point boundaries)
extern Boundary boundaries[BOUNDARY_MAX];
extern int boundary_num, active_boundary;

// PID information
extern unsigned long time_prev;
extern double integral_l, integral_r;
extern int prev_l, prev_r;
extern int target_l, target_r;	// target time between ticks
extern int out_l, out_r;			// output values

// internal coordinates
extern double x, y, theta, tot_distance;	
extern double to_turn;	// turning in place

// turn in place
extern float turn_size;

// waypoint navigation
extern Target targets[TARGET_MAX];
extern double target_distance, last_target_distance, heading_error;
extern int target;	// target index
extern int process_cycles;

// motor control output hbridges
extern Hbridge l, r;		
// 1 for forward, -1 for backward
extern char dir_l, dir_r;

// line detection for navigation correction
extern unsigned long time_prev_sensors;
extern int sensor_num;
extern byte sensors[SENSOR_MAX];
extern byte indicators[SENSOR_MAX];
extern byte on_lines;	// bit array, only 8 bits (sensors)
extern byte prev_on_lines;
extern int thresholds[SENSOR_MAX];


extern bool drive, paused, on;

// methods
void tick_left();
void tick_right();

// previous cycle on line
bool prev_on_line(byte pin);
bool on_line(byte pin);

bool is_intersection(int x, int y);
// restrict values between low and high limits
void clamp(int& parameter, int low, int high);
void pid_control(int tl, int tr);
// update internal position and call speed control
void odometry();
// target acquisition
void locate_target();
// navigating to target
void navigate();
// what to do when arriving at 
void waypoint(byte layer);
// turning at waypoints
void hard_turn();
// avoiding boundaries
void avoid_boundary();



// decide which is the highest active layer to pass to motor control
void arbitrate();
void motor_control(byte layer);


// user insertable behaviour
void user_behaviours();
void user_waypoint();
void user_correct();
void user_start();
void user_stop();


// 3 pins each for hbridges, by default off and going forward
void initialize_robot(byte c1_l, byte c2_l, byte outpin_l, byte c1_r, byte c2_r, byte outpin_r);

void start(byte layer);
void stop(byte layer);
void hard_break(byte layer);
void resume_drive(byte layer);

byte get_active_layer();
bool allowed_layer(byte layer);
void enable_layer(byte layer);
void disable_layer(byte layer);

void set_coordinate(double tx, double ty, double td = 0.0);
void set_drive(bool mode);

int add_target(double tx, double ty, double td = ANY_THETA, byte type = TARGET_NAV, bool rad = false);
int add_boundary(double bx, double by, double radius = 0);	// by default a point
int add_sensor(byte sensor_pin, byte indicator_pin = -1);	// -1 means no indicator associated with sensor

bool go();	// base user to be called in loop
bool correct(); // correct lines to be called in a loop

// calibrate the sensors
void calibrate();
void indicate_sensors();
void correct_to_grid();
int square_heading();
bool far_from_intersection(int candidate_x, int candidate_y);


// retrieve information from the robot
bool get_on();
double get_x();
double get_y();
double get_theta();

int get_active_target();
const Target& get_target();

int get_direction_l();
int get_direction_r();

double get_target_distance();
double get_heading_error();
double current_distance();


int get_boundary_num();
const Boundary& get_boundary(int b);




}	// end namespace
