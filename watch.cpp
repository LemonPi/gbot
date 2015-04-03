#include "gbot.h"
#include "parameters.h"

namespace robot {


// turn to face the game board
void turn_to_watch() {
	Layer& watch = layers[LAYER_WATCH];
	float to_turn = HALFPI - theta;
	watch.speed = 0;
	if (abs(to_turn) < THETA_TOLERANCE) {
		if (!paused) hard_break(LAYER_WATCH);
		watch.angle = 0;
		return;
	}
	else {
		if (paused) resume_drive(LAYER_WATCH);
		if (to_turn > 0) watch.angle = COR_TURN;
		else watch.angle = -COR_TURN;
	}
}
// poll sensors bar for balls
void watch_balls_drop() {
	if (!layers[LAYER_WATCH].active) return;

	if (paused && active_layer == LAYER_WATCH && millis() - last_calibrate_time > CALLIBRATION_TIME*4) {
		SERIAL_PRINTLN("CB");
		calibrate_bar(2000);
		last_calibrate_time = millis();
	}

	turn_to_watch();
	fire_lasers();
	// check if any ball dropped
	watch_bar();
	// check if only 1 ball dropped (check for power of 2; if not then probably false alarm)
	if ((ball_drops != BALL_LESS) && ((ball_drops & (ball_drops - 1)) == 0)) {
		// need to be observe dropping in the same column for 3 cycles
		if (ball_drops & prev_ball_drops) ++consecutive_drops;
		else consecutive_drops = 0;

		SERIAL_PRINT('d');
		SERIAL_PRINTLN(ball_drops,BIN);

		if (consecutive_drops == REAL_DROP) {
			update_game_board();
		}
	}
	else consecutive_drops = 0;

	drop_off_ball();

}

void update_game_board() {
	// normally assume dropped by opponent
	byte ball_ownership = THEIR_BALL;
	// if just dropped ball, assume ball was ours
	if (ball_status > BALL_LESS) {	// during the count down from BALL_TO_BE_DROPPED
		if (millis() - last_dropped_ours < DROPPED_TOO_RECENTLY) return;
		played_ball = true;
		ball_ownership = OUR_BALL;
		last_dropped_ours = millis();
	}
	else {
		// false alarm if dropped too recently
		if (millis() - last_dropped_theirs < DROPPED_TOO_RECENTLY) return;
		else last_dropped_theirs = millis();
	}


	// update slots and top slots
	for (byte slot = 0; slot < GAME_COLS; ++slot) {
		if (ball_drops & (B1000000 >> slot)) {

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

	SERIAL_PRINTLN("----");
	for (int height = GAME_HEIGHT-1; height >= 0; --height) {
		for (byte slot = 0; slot < GAME_COLS; ++slot) {
			byte this_ball = slots[slot][height];
			if (this_ball == NO_BALL) {SERIAL_PRINT("  ");}
			else if (this_ball == OUR_BALL) {SERIAL_PRINT("O ");}
			else {SERIAL_PRINT("T ");}
		}
		SERIAL_PRINT('\n');
	}
}

void drop_off_ball() {
	// check if received ball from rbot
	// if so, calculate best top slot and go to it (ONLY place where new targets are set)
	if (received_ball() && !layers[LAYER_PLAY].active) {	// ignore when dropping ball
		SERIAL_PRINTLN(ball_status);
		if (ball_status == BALL_LESS) {
			SERIAL_PRINTLN("RB");
			ball_status = JUST_GOT_BALL;
		}
		// enough cycles, assume secured
		else if (ball_status == SECURED_BALL) {
			pick_best_col();

			SERIAL_PRINT('s');	// secured ball
			SERIAL_PRINTLN(best_top_slot);

			ball_status = BALL_TO_BE_DROPPED;
			// at this point ball has not been played
			played_ball = false;

			// go to top best slot
			add_target(col_pos[best_top_slot].x, col_pos[best_top_slot].y, 90, TARGET_PLAY);
			// anticipate relative position
			rel_pos = best_top_slot;
			// stop watching the game when moving around
			layers[LAYER_PLAY].active = true;
			layers[LAYER_WATCH].active = false;
			return;
		}
		else ++ball_status;
	}
}

void fire_lasers() {
	for (byte l = 0; l < bar_num; ++l) {

	}
}

// updates if slots and whether ball dropped
void watch_bar() {
	prev_ball_drops = ball_drops;
	for (int b = COL_4 - rel_pos; b < bar_num && (COL_7 - (b + rel_pos - COL_4) >= 0); ++b) {

		bool not_ambient;
		if (b >= 0) not_ambient = abs(analogRead(bars[b]) - ambient[b]) > AMBIENT_THRESHOLD;
		else not_ambient = 0;
		// account for relative position (offset by rendezvous position to 0)
		bitWrite(ball_drops, COL_7 - (b + rel_pos - COL_4), not_ambient);
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

void calibrate_bar(unsigned long duration) {
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
    while ((millis() - calibrate_start) < duration) {
    	++n;
        for (byte pin = 0; pin < BAR_MAX; ++pin) {
        	readings[pin] += analogRead(bars[pin]);
        }
    }

    // set threshold to be average (anything threshold away from it is non-ambient)
    for (byte pin = 0; pin < BAR_MAX; ++pin) 
        ambient[pin] = (float)readings[pin] / (float)n;

    for (byte pin = 0; pin < BAR_MAX; ++pin) {
    	SERIAL_PRINT(ambient[pin]);
    	SERIAL_PRINT(' ');
    }
    SERIAL_PRINT('\n');
}

bool received_ball() {
	return digitalRead(ball_pin);
}

}