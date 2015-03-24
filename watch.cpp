#include "gbot.h"
#include "parameters.h"

namespace robot {

// poll sensors bar for balls
void watch_balls_drop() {
	if (!layers[LAYER_WATCH].active) return;

	// check if received ball from rbot
	// if so, calculate best top slot and go to it (ONLY place where new targets are set)
	if (received_ball() && !layers[LAYER_PLAY].active) {	// ignore when dropping ball
		if (ball_status == BALL_LESS) {
			ball_status = JUST_GOT_BALL;
		}
		// enough cycles, assume secured
		else if (ball_status == SECURED_BALL) {
			pick_best_col();

			ball_status = BALL_TO_BE_DROPPED;
			played_ball = false;

			// go to top best slot
			add_target(col_pos[best_top_slot].x, col_pos[best_top_slot].y, 90, TARGET_PLAY);
			// anticipate relative position
			rel_pos = best_top_slot;
			// stop watching the game when moving around
			layers[LAYER_PLAY].active = true;
			layers[LAYER_WATCH].active = false;
		}
		else ++ball_status;
	}


	fire_lasers();
	// check if any ball dropped
	watch_bar();

	SERIAL_PRINT('d');
	SERIAL_PRINTLN(ball_drops,BIN);

	// check if only 1 ball dropped (check for power of 2; if not then probably false alarm)
	if (ball_drops != BALL_LESS && ball_drops & (ball_drops - 1) == 0) {
		
		byte ball_ownership = NO_BALL;

		// if just dropped ball, assume ball was ours
		if (ball_status > BALL_LESS) {	// during the count down from BALL_TO_BE_DROPPED
			played_ball = true;
			ball_ownership = OUR_BALL;
		}
		// at any other time means dropped by opponent
		else {
			ball_ownership = THEIR_BALL;
		}

		// update slots and top slots
		for (byte slot = 0; slot < 8; ++slot) {
			if (played_ball & (1 << slot)) {

				slots[slot][top_slots[slot].h] = ball_ownership;
				++top_slots[slot].h;
				// might have to REMOVE prints as it's serial printing inside loop
				if (ball_ownership == OUR_BALL) {SERIAL_PRINT('O');}
				else {SERIAL_PRINT('T');}
				SERIAL_PRINT(slot);
				SERIAL_PRINT(' ');
				SERIAL_PRINTLN(top_slots[slot].h);
				break;
			}
		}

	}

}

void fire_lasers() {
	for (byte l = 0; l < bar_num; ++l) {

	}
}

// updates if slots and whether ball dropped
void watch_bar() {
	for (byte b = 0; b < bar_num && BAR_MAX - (b + rel_pos - COL_4) >= 0; ++b) {
		bool not_ambient = analogRead(bars[b]) - ambient[b] > AMBIENT_THRESHOLD;
		// account for relative position (offset by rendezvous position to 0)
		bitWrite(ball_drops, BAR_MAX - (b + rel_pos - COL_4), not_ambient);
	}
}

int add_bar_sensor(byte sensor_pin, byte laser_pin) {
	if (bar_num < BAR_MAX) {
		bars[bar_num] = sensor_pin;
		lasers[bar_num] = laser_pin;
		++bar_num;
	}
	return bar_num;
}

void calibrate_bar() {
	if (bar_num != BAR_MAX) {
		SERIAL_PRINT("Not all bar sensors added: "); 
		SERIAL_PRINTLN(bar_num); 
		return;
	}

	// number of trials
	unsigned long n = 0; 
    unsigned long readings[BAR_MAX] = {0};

    unsigned long calibrate_start = millis();
    // calibrate all the pins
    while ((millis() - calibrate_start) < CALLIBRATION_TIME) {
        for (byte pin = 0; pin < BAR_MAX; ++pin) {
        	++n;
        	readings[pin] += analogRead(bars[pin]);
        }
    }

    // set threshold to be average (anything threshold away from it is non-ambient)
    for (byte pin = 0; pin < BAR_MAX; ++pin) 
        ambient[pin] = (float)readings[pin] / (float)n;
}

bool received_ball() {
	return digitalRead(ball_pin);
}

}