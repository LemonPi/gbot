#include <Arduino.h>
#include <hbridge.h>	// assumes motors are controlled by hbridges
#include <robot.h>
#include "parameters.h"

namespace robot {

volatile int tick_l = 0;
volatile int tick_r = 0;		// time between ticks
int instant_tick_l, instant_tick_r;
// subsumption layers
Layer layers[LAYER_NUM];
byte active_layer;
byte allowed_layers;

// avoid boundaries (point boundaries)
Boundary boundaries[BOUNDARY_MAX];
int boundary_num, active_boundary;

// PID information
unsigned long time_prev;
double integral_l, integral_r;
int prev_l, prev_r;
int target_l, target_r;	// target time between ticks
int out_l, out_r;			// output values

// internal coordinates
double x, y, theta, tot_distance;	
double to_turn;

// turning in place
float turn_size;

// waypoint navigation
Target targets[TARGET_MAX];
double target_distance, last_target_distance, heading_error;
int target;	// target index
int process_cycles;

// motor control output hbridges
Hbridge l, r;		
// 1 for forward, -1 for backward
char dir_l, dir_r;

// line detection for navigation correction
unsigned long time_prev_sensors;
int sensor_num;
byte sensors[SENSOR_MAX];
byte indicators[SENSOR_MAX];
byte on_lines, prev_on_lines;
int thresholds[SENSOR_MAX];


bool drive, on, paused;

const double kp = KP;
const double ki = KI * (CYCLE_TIME / 1000.0);
const double kd = KD / (CYCLE_TIME / 1000.0);

void tick_left() {++tick_l;}
void tick_right() {++tick_r;}

// called every loop, subsumption system
bool go() {
	if (!on) return false;
	unsigned long now = millis();
	unsigned long elapsed = now - time_prev;
	if (now - time_prev < CYCLE_TIME) return false;

	process_cycles = 1;
	// update internal position and do speed control
	odometry();


	// might have to reprocess cycle if waypoint is reached
	while (process_cycles > 0) {
		// calculate target_distance and target_theta
		locate_target();	
		// steer and set speed based on target and current position and heading
		if (allowed_layer(LAYER_NAV)) navigate();
		// turn in place at waypoints if necessary
		if (allowed_layer(LAYER_BOUND)) avoid_boundary();
		if (allowed_layer(LAYER_TURN)) hard_turn();

		user_behaviours();

		arbitrate();
		--process_cycles;
	}

	pid_control(instant_tick_l, instant_tick_r);

	if (drive != MANUAL) {
		l.drive(out_l);
		r.drive(out_r);
	}

	time_prev = now;

	return true;
}

// called every loop, position correction system
bool correct() {
	unsigned long now = millis();
	if (now - time_prev_sensors < SENSOR_TIME) return false;

	// also updates readings
	indicate_sensors();

	if (!on) return false;
	// corrects internal positioning
	user_correct();

	time_prev_sensors = now;
	return true;
}

void initialize_robot(byte c1_l, byte c2_l, byte outpin_l, byte c1_r, byte c2_r, byte outpin_r) {
	instant_tick_l = instant_tick_r = 0;
	boundary_num = 0;
	active_boundary = NONE_ACTIVE;
	allowed_layers = ALL_ALLOWED;
	// PID speed control parameters
	time_prev = 0;
	integral_l = integral_r = 0;
	prev_l = prev_r = 0;
	target_l = target_r = START_SPEED; 
	out_l = out_r = 100;

	// internal positioning and navigation
	x = y = theta = tot_distance = to_turn = 0;
	target_distance = last_target_distance = heading_error = 0;
	target = NONE_ACTIVE;
	process_cycles = 1;

	turn_size = 0;

	// motor control
	l.pin_assign(c1_l, c2_l, outpin_l);
	r.pin_assign(c1_r, c2_r, outpin_r);
	dir_l = dir_r = FORWARD;

	// line detection and other sensor related behaviours
	time_prev_sensors = 0;
	sensor_num = 0;


	drive = AUTOMATIC;
	on = false; 
	paused = true;

	// initial setup
	for (byte l = 0; l < LAYER_NUM; ++l) layers[l].active = false;

	attachInterrupt(0, tick_left, FALLING);
	attachInterrupt(1, tick_right, FALLING);
	// initially stopped
	l.stop();
	r.stop();

	// bottom wait layer, always active, 0 speed
	layers[LAYER_WAIT].active = true;
}

void clamp(int& parameter, int low, int high) {
	if (parameter > high) parameter = high;
	else if (parameter < low) parameter = low;	
}

bool is_intersection(int x, int y) {
	return (x % GRID_WIDTH) & (y % GRID_WIDTH) == 0;
}

void start(byte layer) {
	resume_drive(layer);
	on = true;
	tick_l = tick_r = 0;
	if (target != NONE_ACTIVE && allowed_layer(LAYER_NAV)) {
		layers[LAYER_NAV].active = true;
	}
	else waypoint(100);
	user_start();
}

void stop(byte layer) {
	on = false;
	hard_break(layer);
	user_stop();
}

void hard_break(byte layer) {	
	paused = true;
	l.stop(); 
	r.stop();
	SERIAL_PRINT('h');	// hard break
	SERIAL_PRINTLN(layer);
}
void resume_drive(byte layer) {
	paused = false;
	if (dir_l == FORWARD) l.forward();
	else l.backward();
	if (dir_r == FORWARD) r.forward();
	else r.backward();
	SERIAL_PRINT('r');	// resume
	SERIAL_PRINTLN(layer);	// resume
}

void pid_control(int tl, int tr) {
	int error_l = target_l - tl;
	int error_r = target_r - tr;

	// derivative on measurement to fix derivative kick
	int input_change_l = (tl - prev_l);
	int input_change_r = (tr - prev_r);

	// update state
	prev_l = tl;
	prev_r = tr;

	// integral
	integral_l += ki * error_l;
	integral_r += ki * error_r;
	clamp((int&)integral_l, 0, 255);
	clamp((int&)integral_r, 0, 255);

	out_l = kp*error_l + integral_l - kd*input_change_l;
	out_r = kp*error_r + integral_r - kd*input_change_r;

	clamp(out_l, 40, 250);
	clamp(out_r, 40, 250);
}

void arbitrate() {
	// loop through layers, pass highest priority (first) requested behaviour to motor control	
	for (byte l = 0; l < LAYER_NUM; ++l) {
		if (l == LAYER_WAIT && !paused) {active_layer = l; hard_break(LAYER_WAIT);}	
		else if (layers[l].active) {
			motor_control(l);
			active_layer = l;
			return;
		}	
	}
}

byte get_active_layer() {
	return active_layer;		
}

bool allowed_layer(byte layer) {
	return allowed_layers & (1 << layer);
}
void enable_layer(byte layer) {
	bitSet(allowed_layers, layer);
}
void disable_layer(byte layer) {
	layers[layer].active = false;
	bitClear(allowed_layers, layer);
}

// updates internal position
void odometry() {
	// reset for next round and do speed control
	instant_tick_l = tick_l;
	tick_l = 0;
	instant_tick_r = tick_r;
	tick_r = 0;

	// update internal position
	// displacements in mm
	double displacement_l = dir_l * (double)instant_tick_l * MM_PER_TICK_L;
	double displacement_r = dir_r * (double)instant_tick_r * MM_PER_TICK_R;
	double displacement = (displacement_l + displacement_r) * 0.5;

	// total distance is a scalar
	if (displacement > 0) tot_distance += displacement;
	else tot_distance -= displacement;

	theta += atan2(displacement_l - displacement_r, BASE_WIDTH);
	x += displacement * cos(theta);
	y += displacement * sin(theta);
	// keep theta within [-180,180]
	if (theta > PI) theta -= TWOPI;
	else if (theta < -PI) theta += TWOPI;
}


// with an arbitrated layer, sets the target_l and target_r as well as direction
void motor_control(byte layer) {
	const Layer& control_layer = layers[layer];

	// sets the target for PID to control speed to
	target_l = control_layer.speed + control_layer.angle;
	if (target_l < 0) {
		target_l = -target_l;
		if (dir_l == FORWARD) {
			// store change of direction so that drift is accounted for
			dir_l = BACKWARD;
			l.backward();
		}
	}
	else if (target_l > 0 && dir_l == BACKWARD) {
		dir_l = FORWARD;
		l.forward();
	}

	target_r = control_layer.speed - control_layer.angle;
	if (target_r < 0) {
		target_r = -target_r;
		if (dir_r == FORWARD) {
			dir_r = BACKWARD;
			r.backward();
		}
	}
	else if (target_r > 0 && dir_r == BACKWARD) {
		dir_r = FORWARD;
		r.forward();
	}
	// target should always be positive
}

void set_coordinate(double tx, double ty, double td) {
	x = tx;
	y = ty;
	theta = td*DEGS;
}

void set_drive(bool mode) {drive = mode;}

bool get_on() {return on;}
double get_x() {return x;}
double get_y() {return y;}
double get_theta() {return theta;}

int get_active_target() {return target;}
const Target& get_target() {if (target == NONE_ACTIVE) return targets[0]; return targets[target];}

int get_direction_l() {return dir_l;}
int get_direction_r() {return dir_r;}

double get_target_distance() {return target_distance;}
double get_heading_error() {return heading_error;}

int get_boundary_num() {return boundary_num;}
const Boundary& get_boundary(int b) {if (b >= boundary_num) b = 0; return boundaries[b];} 

}