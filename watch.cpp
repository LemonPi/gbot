#include "gbot.h"
#include "parameters.h"

namespace robot {


// turn to face the game board (only ever active if watch is active)
void turn_to_watch() {
	// control whatever the active layer is
	// SERIAL_PRINTLN('.');
	Layer& watch = layers[active_layer];
	if (active_layer == LAYER_WATCH) watch.angle = 0; 

	float to_turn = HALFPI - theta;
	// kick whenever angle off by too much
	if (abs(to_turn) > 2*THETA_TOLERANCE) {
		if (paused) resume_drive(LAYER_WATCH);
		turn_factor = 1;
	}
	else if (turned_to_watch > RELIABLE_CORRECT_CYCLE) {	
		turn_factor = 1;
		return;
	}


	int turn_speed = COR_TURN*turn_factor;
	// jump start turning
	if (turn_factor == 1) turn_speed *= 1.5;
	// stationary active layer, turn in place
	if (watch.speed == 0) {
		if (abs(to_turn) < THETA_TOLERANCE) {
			if (!paused) hard_break(LAYER_WATCH);
			watch.angle = 0;
			++turned_to_watch;
		}
		else {
			turned_to_watch = 0;
			SERIAL_PRINTLN("TTW");
			if (paused) resume_drive(LAYER_WATCH);
			if (to_turn > 0) watch.angle = 2*turn_speed;
			else watch.angle = -2*turn_speed;
		}
	}
	// correct more gradually while moving
	else {
		if (abs(to_turn) < THETA_TOLERANCE) {
			// watch.angle += 0;
			++turned_to_watch;
		}
		else {
			turned_to_watch = 0;
			SERIAL_PRINTLN("TTM");
			if (to_turn > 0) watch.angle = turn_speed;
			else watch.angle = -turn_speed;
		}
	}
	turn_factor *= 0.9;
}
// poll sensors bar for balls
void watch_balls_drop() {
	if (!layers[LAYER_WATCH].active) {
		// SERIAL_PRINTLN("NW");
		fire_lasers(LOW);
		return;
	}


	// // try shimmying if far from desired x and recently played ball
	// if (allowed_layer(LAYER_SHIMMY) && !layers[LAYER_PLAY].active && 
	// 	(abs(x - RENDEZVOUS_X) > 2*RENDEZVOUS_CLOSE) && 
	// 	(last_lift == 0 || millis() - last_lift < RECENT_LIFT)) {
	// 	// disable_layer(LAYER_COR);
	// 	disable_layer(LAYER_WATCH);
	// 	resume_drive(LAYER_WATCH);
	// 	layers[LAYER_PLAY].active = true;
	// 	SERIAL_PRINTLN("sshim");
	// 	// 100mm to the left
	// 	add_target((x + RENDEZVOUS_X)/2, y - 300, 90, TARGET_SHIMMY);
	// 	++process_cycles;
	// 	return;
	// }


	if (!layers[LAYER_PLAY].active && millis() - last_random_lift > CALLIBRATION_TIME*2) {
		SERIAL_PRINTLN("RL");
		lift_ball();
		delay(500);
		stop_lift_ball();
		last_random_lift = millis();
	}
	if (turned_to_watch < RELIABLE_CORRECT_CYCLE) return;
	// only watch if you don't have ball or about to drop ball?
	if (millis() - last_calibrate_time > CALLIBRATION_TIME*3) {
		SERIAL_PRINTLN("CB");
		calibrate_bar(1000);
	}
	fire_lasers(HIGH);
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

			SERIAL_PRINT("top");	// secured ball
			SERIAL_PRINTLN(best_top_slot);

			ball_status = BALL_TO_BE_DROPPED;
			// at this point ball has not been played
			played_ball = false;
			// go to top best slot
			add_target(col_pos[best_top_slot].x, col_pos[best_top_slot].y, 90, TARGET_PLAY);
			// stop watching the game when moving around
			disable_layer(LAYER_WATCH);
			// disable_layer(LAYER_COR);
			layers[LAYER_PLAY].active = true;
			return;
		}
		else if (ball_status < SECURED_BALL) ++ball_status;
	}
	else if (!received_ball() && !layers[LAYER_PLAY].active && ball_status < SECURED_BALL) ball_status = BALL_LESS;
}

void fire_lasers(int level) {
	// SERIAL_PRINT("FL");
	// SERIAL_PRINT(laser_pin);
	// SERIAL_PRINT('|');
	// SERIAL_PRINTLN(level);

	if (level == 0) digitalWrite(laser_pin, LOW);
	else digitalWrite(laser_pin, HIGH);
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
	last_calibrate_time = millis();
}

bool received_ball() {
	return digitalRead(ball_pin);
}

}