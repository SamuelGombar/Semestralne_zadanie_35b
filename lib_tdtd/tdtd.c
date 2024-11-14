/*
 * tdtd.c
 *
 *  Created on: Nov 14, 2024
 *      Author: Michal Zborovjan
 *
 *      Translate Data To Decision
 */

#include "tdtd.h"
#include <stdint.h>

enum decision_type make_decision(double left, double center, double right){
    enum decision_type decision = forward;

    uint8_t left_val;
    uint8_t center_val;
    uint8_t right_val;
    uint8_t case_constant;

    if (left > DIST_LONG) left_val = NO_WORRY;
    else if (left > DIST_SHORT) left_val = LEFT_VAL_WORRY;
    else left_val = LEFT_VAL_PANIC;

    if (center > DIST_LONG) center_val = NO_WORRY;
    else if (center > DIST_SHORT) center_val = CENTER_VAL_WORRY;
    else center_val = CENTER_VAL_PANIC;

    if (right > DIST_LONG) right_val = NO_WORRY;
    else if (right > DIST_SHORT) right_val = RIGHT_VAL_WORRY;
    else right_val = RIGHT_VAL_PANIC;

    case_constant = left_val | center_val | right_val;

    switch(case_constant){
        case (LEFT_VAL_PANIC | CENTER_VAL_PANIC | RIGHT_VAL_PANIC): decision = backward; break;
        case (LEFT_VAL_WORRY | CENTER_VAL_WORRY | RIGHT_VAL_WORRY): decision = forward; break;
        case (NO_WORRY): decision = forward; break;


        case (LEFT_VAL_PANIC | CENTER_VAL_PANIC | RIGHT_VAL_WORRY): decision = strong_right; break;
        case (LEFT_VAL_PANIC | CENTER_VAL_WORRY | RIGHT_VAL_PANIC): decision = backward; break;
        case (LEFT_VAL_WORRY | CENTER_VAL_PANIC | RIGHT_VAL_PANIC): decision = strong_left; break;
        case (LEFT_VAL_PANIC | CENTER_VAL_WORRY | RIGHT_VAL_WORRY): decision = strong_right; break;
        case (LEFT_VAL_WORRY | CENTER_VAL_WORRY | RIGHT_VAL_PANIC): decision = strong_left; break;
        case (LEFT_VAL_WORRY | CENTER_VAL_PANIC | RIGHT_VAL_WORRY): decision = backward; break;


        case (LEFT_VAL_PANIC | CENTER_VAL_PANIC): decision = strong_right; break;
        case (LEFT_VAL_PANIC | CENTER_VAL_WORRY): decision = strong_right; break;
        case (LEFT_VAL_WORRY | CENTER_VAL_PANIC): decision = strong_right; break;
        case (LEFT_VAL_WORRY | CENTER_VAL_WORRY): decision = weak_right; break;

        case (CENTER_VAL_PANIC | RIGHT_VAL_PANIC): decision = strong_left; break;
        case (CENTER_VAL_PANIC | RIGHT_VAL_WORRY): decision = strong_left; break;
        case (CENTER_VAL_WORRY | RIGHT_VAL_PANIC): decision = strong_left; break;
        case (CENTER_VAL_WORRY | RIGHT_VAL_WORRY): decision = weak_left; break;

        case (LEFT_VAL_PANIC | RIGHT_VAL_PANIC): decision = forward; break;
        case (LEFT_VAL_PANIC | RIGHT_VAL_WORRY): decision = forward; break;
        case (LEFT_VAL_WORRY | RIGHT_VAL_PANIC): decision = forward; break;
        case (LEFT_VAL_WORRY | RIGHT_VAL_WORRY): decision = forward; break;

        case (LEFT_VAL_PANIC): decision = strong_right; break;
        case (CENTER_VAL_PANIC): decision = strong_right; break;
        case (RIGHT_VAL_PANIC): decision = strong_left; break;

        case (LEFT_VAL_WORRY): decision = weak_right; break;
        case (CENTER_VAL_WORRY): decision = weak_right; break;
        case (RIGHT_VAL_WORRY): decision = weak_left; break;

        default: decision = backward;
    }


    return decision;
}


