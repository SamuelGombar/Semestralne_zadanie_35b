/*
 * tdtd.h
 *
 *  Created on: Nov 14, 2024
 *      Author: Michal Zborovjan
 *
 *      Translate Data To Decision
 */

#ifndef TDTD_H_INCLUDED
    #define TDTD_H_INCLUDED

    #define DIST_LONG               1200
    #define DIST_SHORT               400

    #define NO_WORRY                   0
    #define LEFT_VAL_PANIC      (1 << 5)
    #define LEFT_VAL_WORRY      (1 << 4)
    #define CENTER_VAL_PANIC    (1 << 3)
    #define CENTER_VAL_WORRY    (1 << 2)
    #define RIGHT_VAL_PANIC     (1 << 1)
    #define RIGHT_VAL_WORRY     (1 << 0)

#endif // TDTD_H_INCLUDED


enum decision_type{
    forward = 0,
    backward = 1,
    strong_left = 2,
    weak_left = 3,
    strong_right = 4,
    weak_right = 5
};

enum decision_type make_decision(double left, double center, double right);
