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
		float distance[SONAR_MAX];
		for (byte sonar = 0; sonar < sonar_num; ++sonar) {
			// manual pulse
			digitalWrite(trigs[sonar], LOW);
			delayMicroseconds(2);
			digitalWrite(trigs[sonar], HIGH);
			delayMicroseconds(10); 
			digitalWrite(trigs[sonar], LOW);

			long duration = pulseIn(echos[sonar], HIGH);
			distance[sonar] = ((float)duration*0.5) * DISTANCE_FROM_PULSE;
			// SERIAL_PRINT(distance[sonar]);
			// SERIAL_PRINT(' ');
			// can't be too far absolutely or from the previous reading
			if (distance[sonar] > SONAR_TOO_FAR
			 || ((prev_wall_distance[sonar] != 0) && (abs(distance[sonar] - prev_wall_distance[sonar]) > SONAR_CHANGE_ALLOWANCE))
			 ) {
				outlier = true;
				SERIAL_PRINTLN("OUT");
				// backtrack to remove the entire cycle
				for (byte prev_sonar = 0; prev_sonar < sonar; ++prev_sonar)
					wall_distance[prev_sonar] -= distance[prev_sonar];
				break;
			}
			wall_distance[sonar] += distance[sonar];
			// don't delay on the last sonar
			delay(1);
		}
		if (!outlier) ++sonar_cycle;
		outlier = false;		
	}
	else if (sonar_cycle == SONAR_CYCLE) {
		// reset down to 0 by whichever uses the distances
		sonar_cycle = WALL_DISTANCE_READY;
		for (byte sonar = 0; sonar < sonar_num; ++sonar) {
			wall_distance[sonar] *= 1/(float)SONAR_CYCLE;
			wall_distance[sonar] += wall_distance_offset[sonar];
		} 
	}
	// else sonar_cycle = 0;
}

void reset_wall_distance() {
	sonar_cycle = 0;
	for (byte sonar = 0; sonar < sonar_num; ++sonar) 
		prev_wall_distance[sonar] = 0;
}

// modify the angle of the top layer to turn away or towards a wall
void hug_wall() {
	sonar_cycle = 0;
	// overturned, turn in other direction "leaky integrator"
	float distance_offset = wall_distance[SIDE_FRONT] - wall_distance[SIDE_BACK];
	float theta_offset = atan2(distance_offset, SIDE_SONAR_DISTANCE);
	SERIAL_PRINT('H');
	SERIAL_PRINT(wall_distance[SIDE_FRONT]);
	SERIAL_PRINT(' ');
	SERIAL_PRINTLN(wall_distance[SIDE_BACK]);

	int perpendicular_angle = square_heading();

	if (abs(theta_offset < CAN_TURN_IN_PLACE)) {
		if (abs(theta_offset) < THETA_TOLERANCE) {
			theta = perpendicular_angle * DEGS;
		}
		else theta = (perpendicular_angle*DEGS) + theta_offset;
		// at gameboard
		float center_distance = cos(theta_offset)*(wall_distance[SIDE_BACK]*(1-SIDE_FRONT_BACK_RATIO) + wall_distance[SIDE_FRONT]*(SIDE_FRONT_BACK_RATIO) + CENTER_TO_SONAR_DISTANCE);
		// SERIAL_PRINT(wall_distance[SIDE_FRONT]);
		// SERIAL_PRINT(' ');
		// SERIAL_PRINT(wall_distance[SIDE_BACK]);
		// SERIAL_PRINT(' ');
		// SERIAL_PRINTLN(center_distance);
		if (perpendicular_angle == DIR_RIGHT) {
			x = GAME_BOARD_X - center_distance;
			++corrected_x;
		}
		else if (perpendicular_angle == DIR_UP) {
			y = 0 + center_distance;
			++corrected_y;
		}
		else if (perpendicular_angle == DIR_LEFT)
			x = 0 + center_distance;
	}
	// offset is too large, ignore

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
		else if (counted_lines >= LINES_PER_CORRECT && far_from_intersection_gbot(x, y)) {
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

bool far_from_intersection_gbot(int xx, int yy) {
	// consider red line as lines as usual
	byte offset_x = abs((int)xx) % GRID_WIDTH;
	byte offset_y = abs((int)yy) % GRID_WIDTH;
	return (offset_x < INTERSECTION_TOO_CLOSE || offset_x > GRID_WIDTH - INTERSECTION_TOO_CLOSE) ^
			(offset_y < INTERSECTION_TOO_CLOSE || offset_y > GRID_WIDTH - INTERSECTION_TOO_CLOSE);	
}




}