#include <Arduino.h>
#include <robot.h>
#include "parameters.h"

namespace robot {

int add_boundary(double bx, double by, double radius) {
	if (boundary_num < BOUNDARY_MAX) {
		boundaries[boundary_num].x = bx;
		boundaries[boundary_num].y = by;
		boundaries[boundary_num].r = radius;
		++boundary_num;
		return boundary_num;		
	}
	return -1;
}

void avoid_boundary() {
	Layer& bound = layers[LAYER_BOUND];

	// deactivate boundary when far enough
	if (active_boundary != NONE_ACTIVE && boundaries[active_boundary].distance > BOUNDARY_FAR_ENOUGH) {
		active_boundary = NONE_ACTIVE;
		bound.active = false;
	}

	for (int b = 0; b < boundary_num; ++b) {
		Boundary& boundary = boundaries[b];
		// check distance to boundary
		double diff_x = boundary.x - x;
		double diff_y = boundary.y - y;
		// approximate each boundary as circle, from center to point - radius
		boundary.distance = sqrt(sq(diff_x) + sq(diff_y)) - boundary.r;
		if (boundary.distance < 0) { boundary.distance = 0; boundary.threat = EXISTENTIAL_THREAT; }
		// compare this with theta to see if collision likely
		boundary.theta = atan2(diff_y, diff_x);

		double boundary_heading_error = boundary.theta - theta;

		if (boundary.distance < BOUNDARY_TOO_CLOSE &&
			(abs(boundary_heading_error) < BOUNDARY_TOLERANCE)) {

			// high threat comes from being closer and a straight hit
			boundary.threat = (BOUNDARY_TOO_CLOSE - boundary.distance) * 
								(BOUNDARY_TOLERANCE - abs(boundary_heading_error)) / BOUNDARY_TOLERANCE;
	
		}
		// no threat, either angle not a concern or too far away
		else boundary.threat = 0;
	}

	// don't avoid boundary if you're turning in place or close to target
	if (layers[LAYER_TURN].active || target_distance < TARGET_IMMEDIATE || 
		targets[target].type == TARGET_GET || layers[LAYER_GET].active) return;
	
	// take care of the case when all boundaries are inactive
	double max_threat = 0;

	for (int b = 0; b < boundary_num; ++b) {
		if (boundaries[b].threat > max_threat) {
			active_boundary = b;
			max_threat = boundaries[b].threat;
		}		
	}

	// // if there is an active boundary and enough of a threat
	if (active_boundary != NONE_ACTIVE) {
		if (boundaries[active_boundary].threat >= EXISTENTIAL_THREAT) bound.active = true;

		bound.speed = TOP_SPEED * 0.5;
		// want to keep boundary at +- 90 degrees to hug around it
		// if boundary_heading_error > 0, robot is on left of boundary
		double boundary_heading_error = boundaries[active_boundary].theta - theta;
		if (boundary_heading_error > 0) {
			if (boundary_heading_error > HALFPI) bound.angle = BOUND_TURN;
			else bound.angle = -BOUND_TURN;
		}
		else {
			if (boundary_heading_error < -HALFPI) bound.angle = -BOUND_TURN;
			else bound.angle = BOUND_TURN;
		}
	}	

}

}