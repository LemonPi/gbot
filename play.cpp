#include "gbot.h"
#include "parameters.h"

namespace robot {

// play behaviour called in go loop; goes between column positions via backward and forward motion
void play_ball() {
	Layer& play = layers[LAYER_PLAY];
	if (!play.active) return;
	// just don't...
	layers[LAYER_TURN].active = false;

	SERIAL_PRINTLN('p');
	
	play.angle = 0;

	// not watching means still moving, either to column to deposit ball or back to position
	if (!layers[LAYER_WATCH].active) {
		int heading = square_heading();
		if (heading != DIR_RIGHT) return;

		if (paused) resume_drive();

		// only need to consider heading right
		if (targets[target].y > y) play.speed = PLAY_SPEED;
		else play.speed = -PLAY_SPEED;
		// backup without turning (avoid errors)

		// only consider y when playing back and forth
		if (abs(targets[target].y - y) < COL_CLOSE) {

			// if returning to the rendezvous point can simply stop here and switch to watch mode
			if (targets[target].type == TARGET_WATCH && ball_dropped) {
				SERIAL_PRINTLN('R');
				// rendezvous location loads to col 4
				rel_pos = COL_4;
				layers[LAYER_PLAY].active = false;
				waypoint();
			}

			// else must be to deposit a ball
			else {
				SERIAL_PRINTLN('z');
				// listen for self drop
				layers[LAYER_WATCH].active = true;
				ball_dropped = false;
				hard_break();
			}
		}
	}
	else {
		play.speed = 0;
		// ball is missing from bottom and sensors detect a ball dropped; indication can move back
		if (!received_ball() && ball_status == BALL_LESS || played_ball) {
			stop_lift_ball();
			SERIAL_PRINTLN("ret");
			// change the target to be rendezvous point and to watch
			targets[target].x = RENDEZVOUS_X;
			targets[target].y = RENDEZVOUS_Y;
			targets[target].type = TARGET_WATCH;
			played_ball = true;
			jammed = false;
			layers[LAYER_WATCH].active = false;
		}
		// perhaps jammed, try again (counted down all the way to 0)
		else if (received_ball() && ball_status > BALL_LESS && ball_status < 0.5*BALL_TO_BE_DROPPED) {
			SERIAL_PRINTLN('j');
			stop_lift_ball();
			delay(50);
			ball_status = BALL_TO_BE_DROPPED * 0.5;
			jammed = true;
		}
		else {
			layers[LAYER_WATCH].active = true;
			if (jammed) lift_ball_harder();
			else lift_ball();
			--ball_status;
		}
	}

}

void lift_ball() {
	digitalWrite(lift_pin, LIFT_SPEED);
}
void lift_ball_harder() {
	digitalWrite(lift_pin, LIFT_SPEED+30);
}
void stop_lift_ball() {
	digitalWrite(lift_pin, 0);
}

// assign a score to a specific slot (column, height) both starting from 0
int score_top_slot(byte top) {
	score = 0;
	byte col = top_slots[top].c;
	byte height = top_slots[top].h;
	// extend in every direction 

	reset_run();
	// left 
	for (short c = col - 1; streak && c >= 0 && c >= (short)col - SCORE_DEPTH; --c)
		score_slot(c, height);

	reset_run();
	// right
	for (short c = col + 1; streak && c < GAME_COLS && c <= col + SCORE_DEPTH; ++c)
		score_slot(c, height);

	reset_run();
	// down
	for (short h = height - 1; streak && h > 0 && h >= (short)height - SCORE_DEPTH; --h)
		score_slot(col, h);

	reset_run();
	// lower left
	for (short c = col - 1, h = height - 1; streak && h > 0 && h >= (short)height - SCORE_DEPTH &&
		 											  c > 0 && c >= (short)col - SCORE_DEPTH; --c, --h)
		score_slot(c, h);

	reset_run();
	// lower right
	for (short c = col + 1, h = height - 1; streak && h > 0 && h >= (short)height - SCORE_DEPTH &&
		 											  c < GAME_COLS && c <= col + SCORE_DEPTH; ++c, --h)
		score_slot(c, h);

	reset_run();
	// upper left
	for (short c = col - 1, h = height + 1; streak && h < GAME_HEIGHT && h <= height + SCORE_DEPTH &&
		 											  c > 0 && c >= (short)col - SCORE_DEPTH; --c, ++h)
		score_slot(c, h);	

	reset_run();
	// upper right
	for (short c = col + 1, h = height + 1; streak && h < GAME_HEIGHT && h <= height + SCORE_DEPTH &&
		 											  c < GAME_COLS && c <= col + SCORE_DEPTH; ++c, ++h)
		score_slot(c, h);	

	top_slots[top].score = score;
	return score;
}

// resets scoring parameters after a run (along a slot line)
void reset_run() {
	byte continuous_ours = 0;
	byte continuous_theirs = 0;
	bool streak = true;
	byte prev_slot = OUR_BALL;	
}

// updates static membes of the class (assume it's called inside score_board (no parameters to be faster))
void score_slot(byte c, byte h) {
	if (slots[c][h] == NO_BALL) streak = false;
	else if (slots[c][h] == OUR_BALL) {
		// continues our streak
		if (prev_slot == OUR_BALL) {
			++continuous_ours;
			// good chance of scoring if many continuous (will += 4 if connecting 4)
			if (continuous_ours >= THREE_IN_ROW) score += 3; 
			else if (continuous_ours >= TWO_IN_ROW) ++score;
		}
		// streak broken, don't care about additional balls past this point
		else streak = false;
	}
	else {
		// check if it's first one away
		if (prev_slot == THEIR_BALL || (continuous_ours == 0 && continuous_theirs == 0)) {
			++continuous_theirs;
			if (continuous_theirs >= THREE_IN_ROW) score += 2;
			else if (continuous_theirs >= TWO_IN_ROW) ++score;
		}
		else streak = false;
	}	

	prev_slot = slots[c][h];
}

// select best position for gbot to play to
void pick_best_col() {
	int top_score = 0;
	for (byte c = 0; c < GAME_COLS; ++c) {
		score_top_slot(c);
		if (top_slots[c].score > top_score) {
			top_score = top_slots[c].score;
			best_top_slot = c;
		}
	}
}

}