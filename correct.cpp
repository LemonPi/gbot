#include "gbot.h"
#include "parameters.h"

namespace robot {

byte add_sonar(byte t, byte e) {
	if (sonar_num < SONAR_MAX) {
		trigs[sonar_num] = t;
		echos[sonar_num] = e;
		++sonar_num;
		pinMode(t, OUTPUT);
		pinMode(e, INPUT);
	}
	return sonar_num;
}

void touch_wall() {
	if (sonar_cycle < SONAR_CYCLE) {
		bool outlier = false;
		for (byte sonar = 0; sonar < SONAR_MAX; ++sonar) {
			// manual pulse
			digitalWrite(trigs[sonar], LOW);
			delayMicroseconds(2);
			digitalWrite(trigs[sonar], HIGH);
			delayMicroseconds(10); 
			digitalWrite(trigs[sonar], LOW);

			long duration = pulseIn(echos[sonar], HIGH);
			float distance = ((float)duration*0.5) * DISTANCE_FROM_PULSE;
			if (distance > DISTANCE_TOO_FAR) outlier = true;
			else if (!outlier) wall_distance[sonar] += distance;
		}
		if (!outlier) ++sonar_cycle;
		outlier = false;
		
	}
	else if (sonar_cycle == SONAR_CYCLE) {
		// reset down to 0 by whichever uses the distances
		sonar_cycle = WALL_DISTANCE_READY;
		for (byte sonar = 0; sonar < SONAR_MAX; ++sonar) 
			wall_distance[sonar] *= 1/(float)SONAR_CYCLE;
	}
}

// modify the angle of the top layer to turn away or towards a wall
void hug_wall() {
	SERIAL_PRINTLN('H');
	// overturned, turn in other direction "leaky integrator"
	float distance_offset = wall_distance[BACK] - WALL_DISTANCE;
	// back is further away from wall, turn right
	if (distance_offset > SONAR_DISTANCE_TOLERANCE) layers[active_layer].angle = COR_TURN;
	else if (distance_offset < -SONAR_DISTANCE_TOLERANCE) layers[active_layer].angle = -COR_TURN;
	// else {
	// 	theta = (square_heading()*DEGS);
	// 	correct.active = false;
	// 	correct.angle = 0;
	// }

	for (byte sonar = 0; sonar < SONAR_MAX; ++sonar) {
		prev_wall_distance[sonar] = wall_distance[sonar];
		wall_distance[sonar] = 0;
	}
}


void passive_position_correct() {
	if (on_line(CENTER)) {
		++cycles_on_line;
		// activate line following if close enough to target and is on a line
	}
	else {
		// false positive, not on line for enough cycles
		if (cycles_on_line < CYCLES_CROSSING_LINE && cycles_on_line >= 0) ;
		else if (counted_lines >= LINES_PER_CORRECT && far_from_intersection(x, y)) {
			counted_lines = 0;
			// correct whichever one is closer to 0 or 200 
			correct_to_grid();

			SERIAL_PRINTLN('C');
		}
		else {
			++counted_lines;
			SERIAL_PRINTLN('L');
		}
		cycles_on_line = 0;
	}
}


}