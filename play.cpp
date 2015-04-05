#include "gbot.h"
#include "parameters.h"

namespace robot {

// play behaviour called in go loop; goes between column positions via backward and forward motion
void play_ball() {
	Layer& play = layers[LAYER_PLAY];
	if (!play.active) return;
	// just don't...
	layers[LAYER_TURN].active = false;
	

	// not watching means still moving, either to column to deposit ball or back to position
	if (!layers[LAYER_WATCH].active) {
		int heading = square_heading();
		if (heading != DIR_RIGHT) return;

		// only consider y when playing back and forth
		if (abs(targets[target].y - y) < COL_CLOSE) {
			waypoint();
			return;
		}

		// the first point of starting motion, need to turn to watch again
		if (paused) resume_drive(LAYER_PLAY);

		float backing_angle = heading_error;
		// in front of robot
		if (targets[target].y > y) { 
			play.speed = PLAY_SPEED;
		}
		// behind robot
		else {
			play.speed = -PLAY_SPEED;
			// invert angle while going back
			if (backing_angle < 0) backing_angle += PI;
			else backing_angle -= PI;
		}

		if (abs(backing_angle) < THETA_TOLERANCE) play.angle = 0;
		else if (backing_angle < 0) play.angle = -COR_TURN;
		else play.angle = COR_TURN;

		stop_lift_ball();
		// backup without turning (avoid errors)

	}
	// only lift ball if corrected
	else if (paused && turned_to_watch > RELIABLE_CORRECT_CYCLE) {
		play.speed = 0;
		// ball is missing from bottom and sensors detect a ball dropped; indication can move back
		if (!received_ball() && ball_status == BALL_LESS) {
			stop_lift_ball();
			SERIAL_PRINTLN("ret");
			// change the target to be rendezvous point and to watch
			add_target(RENDEZVOUS_X, RENDEZVOUS_Y, 90, TARGET_WATCH);
			++process_cycles;
			enable_layer(LAYER_COR);
			// reset angle to go back (residual from turn to watch)
			play.angle = 0;
			played_ball = true;
			jammed = false;
			// sonar less effective when moving
			disable_layer(LAYER_WATCH);
			// disable_layer(LAYER_COR);
		}
		// perhaps jammed, try again (counted down all the way to 0)
		else if (received_ball() && !jammed && ball_status > BALL_LESS && ball_status < 0.5*BALL_TO_BE_DROPPED) {
			SERIAL_PRINTLN('j');
			stop_lift_ball();
			delay(50);
			ball_status = BALL_TO_BE_DROPPED;
			jammed = true;
		}
		// only lift ball when paused (not turning from turn to watch)
		else {
			// don't try to adjust position while lifting the ball
			disable_layer(LAYER_COR);
			if (jammed) lift_ball_harder();
			else lift_ball();
			--ball_status;
			SERIAL_PRINT('.');
			SERIAL_PRINT(received_ball());
			SERIAL_PRINT('.');
			SERIAL_PRINTLN(ball_status);
		}
	}
}

void lift_ball() {
	analogWrite(lift_pin, LIFT_SPEED);
}
void lift_ball_harder() {
	analogWrite(lift_pin, LIFT_SPEED+30);
}
void stop_lift_ball() {
	analogWrite(lift_pin, 0);
}

// assign a score to a specific slot (column, height) both starting from 0
int score_top_slot(byte top) {
	score = 0;
	byte col = top_slots[top].c;
	byte height = top_slots[top].h;
	// full column
	if (height >= GAME_HEIGHT) {
		top_slots[top].score = -1;
		SERIAL_PRINT("FULL");
		return -1;
	}
	// extend in every direction 

	reset_run();
	// left 
	for (short c = col - 1; streak && c >= 0 && c >= (short)col - SCORE_DEPTH; --c) {
		SERIAL_PRINT('L');
		score_slot(c, height);
	}

	reset_run();
	// right
	for (short c = col + 1; streak && c < GAME_COLS && c <= col + SCORE_DEPTH; ++c) {
		SERIAL_PRINT('R');
		score_slot(c, height);
	}

	reset_run();
	// down
	for (short h = height - 1; streak && h > 0 && h >= (short)height - SCORE_DEPTH; --h) {
		SERIAL_PRINT('D');
		score_slot(col, h);
	}

	reset_run();
	// lower left
	for (short c = col - 1, h = height - 1; streak && h > 0 && h >= (short)height - SCORE_DEPTH &&
		 											  c > 0 && c >= (short)col - SCORE_DEPTH; --c, --h) {
		SERIAL_PRINT("ll");
		score_slot(c, h);
	}

	reset_run();
	// lower right
	for (short c = col + 1, h = height - 1; streak && h > 0 && h >= (short)height - SCORE_DEPTH &&
		 											  c < GAME_COLS && c <= col + SCORE_DEPTH; ++c, --h) {
		SERIAL_PRINT("lr");
		score_slot(c, h);
	}

	reset_run();
	// upper left
	for (short c = col - 1, h = height + 1; streak && h < GAME_HEIGHT && h <= height + SCORE_DEPTH &&
		 											  c > 0 && c >= (short)col - SCORE_DEPTH; --c, ++h) {
		SERIAL_PRINT("ul");
		score_slot(c, h);	
	}

	reset_run();
	// upper right
	for (short c = col + 1, h = height + 1; streak && h < GAME_HEIGHT && h <= height + SCORE_DEPTH &&
		 											  c < GAME_COLS && c <= col + SCORE_DEPTH; ++c, ++h) {	
		SERIAL_PRINT("ur");
		score_slot(c, h);	
	}

	top_slots[top].score = score;
	return score;
}

// resets scoring parameters after a run (along a slot line)
void reset_run() {
	continuous_ours = 0;
	continuous_theirs = 0;
	streak = true;
	prev_slot = OUR_BALL;	
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
	int top_score = top_slots[best_top_slot].score;
	SERIAL_PRINT("prev top:");
	SERIAL_PRINTLN(top_score);
	for (byte c = 0; c < GAME_COLS; ++c) {
		SERIAL_PRINT("c:");
		SERIAL_PRINT(c);
		
		score_top_slot(c);
		if (top_slots[c].score > top_score) {
			top_score = top_slots[c].score;
			best_top_slot = c;
		}
		SERIAL_PRINT('\n');
	}
	for (byte c = 0; c < GAME_COLS; ++c) {
		SERIAL_PRINT(top_slots[c].score);
		SERIAL_PRINT(' ');
	}
	SERIAL_PRINT("best col:");
	SERIAL_PRINTLN(best_top_slot);
	// SERIAL_PRINT('\n');
}

}