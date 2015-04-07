#include <Arduino.h>
#include <hbridge.h>
#include <robot.h>
#include "parameters.h"

namespace robot {

// set target from origin (last reset point)
int add_target(double tx, double ty, double td, byte type, bool rad) {
	if (target + 1 >= TARGET_MAX) return -1;

	if (td != ANY_THETA) {
		if (!rad) {
			td *= DEGS;
		}
		if (td < -PI) td += TWOPI;
		else if (td > PI) td -= TWOPI;
	}
	++target;
	targets[target].x = tx;
	targets[target].y = ty;
	targets[target].theta = td;
	targets[target].type = type;
	if (allowed_layer(LAYER_NAV)) layers[LAYER_NAV].active = true;
	return target;
}

// turning in place
// check if turn has been completed
void hard_turn() {
	// static int stall_cycles = 0;
	Layer& turn = layers[LAYER_TURN];

	turn.speed = 0;				// turn in place, no translational velocity
	if (!turn.active || target == NONE_ACTIVE) { turn.angle = 0; return; }

	to_turn = targets[target].theta - theta;
	// would be faster to turn in the opposite direction 
	if (to_turn > PI) to_turn -= TWOPI;
	else if (to_turn < -PI) to_turn += TWOPI;

	// turn until theta ~= target_theta
	if (abs(to_turn) < THETA_TOLERANCE) {
		SERIAL_PRINTLN("dt");
		// stall_cycles = 0;
		waypoint(LAYER_TURN);
		return;
	}
	
	// either to_turn close to turn size or even greater, kick start it (usually at start of turn)
	if (turn_size - abs(to_turn) < THETA_TOLERANCE) {
		if (to_turn < 0) turn.angle = -KICK_SPEED;
		else turn.angle = KICK_SPEED;
		return;
	}
	// compare against initial turn size
	else if (abs(to_turn) < turn_size) {
		turn.angle = to_turn/turn_size * NAV_TURN * 3;
	}
	else {
		turn.angle = to_turn * NAV_TURN * 4;
		SERIAL_PRINT("ot");
		SERIAL_PRINTLN(to_turn*RADS);
	}

	if (abs(turn.angle) > TOP_SPEED) {
		if (to_turn < 0) turn.angle = -TOP_SPEED;
		else turn.angle = TOP_SPEED;
	}
	else {
		if (to_turn < 0) {turn.angle -= 0.40*MIN_SPEED;}
		else {turn.angle += 0.40*MIN_SPEED;}
	}
}


// get to a target waypoint by 
// resets target_x, target_y when arrived
void navigate() {
	Layer& nav = layers[LAYER_NAV];
	if (!nav.active || layers[LAYER_TURN].active) return;

	// turn in place
	// not too close to active boundary
	if ((active_boundary == NONE_ACTIVE || 	// no active boundaries
			(boundaries[active_boundary].distance > target_distance &&	// target is closer than the boundary
			abs(boundaries[active_boundary].theta - heading_error) > 0.3)) &&
		target_distance > TARGET_IMMEDIATE &&
		drive == AUTOMATIC &&
		abs(heading_error) > CAN_TURN_IN_PLACE &&
		allowed_layer(LAYER_TURN)) { 	// need large enough of a turn)

		// push temporary targets (stationary, but turning)
		add_target(x, y, heading_error + theta, TARGET_TURN, true);
		turn_size = abs(heading_error);
		layers[LAYER_TURN].active = true;
		hard_break(LAYER_NAV, 3);
		nav.active = false;

		SERIAL_PRINT("t ");
		SERIAL_PRINTLN((int)(heading_error*RADS));
		
	}

	// arrived at target (can't get any closer presumably)
	else if ((target_distance < TARGET_IMMEDIATE) ||
			 ((target_distance < TARGET_CIRCLE) && (target_distance > last_target_distance))) {

		SERIAL_PRINTLN('c');
		nav.active = false;	// no longer need to navigate	

		if (abs(targets[target].theta - ANY_THETA) > 1 && 	// target isn't just any theta
			abs(targets[target].theta - theta) > THETA_TOLERANCE &&
			allowed_layer(LAYER_TURN)) {	// still needs turning
			hard_break(LAYER_NAV, 3);
			layers[LAYER_TURN].active = true;
			turn_size = abs(targets[target].theta - theta);
			++process_cycles;
		}
		// don't need to turn anymore
		else {
			SERIAL_PRINTLN('f');
			waypoint(LAYER_NAV);
		}
	}
	// still seeking target
	else {
		// steer heading toward target
		if (abs(heading_error) < THETA_TOLERANCE) nav.angle = 0;
		else {
			if (heading_error < 0) nav.angle = -NAV_TURN;
			else nav.angle = NAV_TURN;
		}
		// slow down closer to target
		if (target_distance < TARGET_CLOSE) 
			nav.speed = TOP_SPEED * (target_distance/TARGET_CLOSE);
		else nav.speed = TOP_SPEED;
		// prevent going backwards/stalling when navigation
		if (nav.speed < NAV_TURN) nav.speed = NAV_TURN;
	}
	last_target_distance = target_distance;
}

double current_distance() {
	// accumulate current ticks
	instant_tick_l = tick_l;
	instant_tick_r = tick_r;
	double displacement_l = dir_l * (double)instant_tick_l * MM_PER_TICK_L;
	double displacement_r = dir_r * (double)instant_tick_r * MM_PER_TICK_R;

	return tot_distance + abs(displacement_l + displacement_r)*0.5;	
}

// updates target_distance and heading_error
void locate_target() {
	if (target < 0) return;
	double diff_x = targets[target].x - x;
	double diff_y = targets[target].y - y;
	double diff_theta = atan2(diff_y, diff_x);
	target_distance = sqrt(sq(diff_x) + sq(diff_y));

	// keep heading error within [-PI, PI]
	heading_error = diff_theta - theta;
	if (heading_error > PI) heading_error -= TWOPI;
	else if (heading_error < -PI) heading_error += TWOPI;
}

// arrived at target, called at the end of other behaviours
// decides what behaviours to enable based on game and current state
void waypoint(byte layer) {
	SERIAL_PRINT('w');
	SERIAL_PRINTLN(layer);
	// wait after turning in place
	if (layer == LAYER_TURN) {
		// SERIAL_PRINTLN("LT");
		hard_break(LAYER_TURN, 5);
	}
	// else {
	// 	SERIAL_PRINT("NT");
	// 	SERIAL_PRINTLN(layer);
	// }
	// other targets to reach, lower stack index, reprocess
	if (target > 0) { 
		--target; 
		++process_cycles; 
		if (allowed_layer(LAYER_NAV)) layers[LAYER_NAV].active = true;
	}
	// reached last target
	else if (target == 0) {
		target = NONE_ACTIVE;
		layers[LAYER_NAV].active = false;
		layers[LAYER_TURN].active = false;
	}

	// get next target by checking current position against game state

	// finished turning in place (2nd call of waypoint)
	if (layers[LAYER_TURN].active) {
		if (allowed_layer(LAYER_NAV)) layers[LAYER_NAV].active = true;
		layers[LAYER_TURN].active = false;
	} 
	
	user_waypoint();
}

}