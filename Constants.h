//
// Created by badru on 17.12.2018.
//

#ifndef CODEBALLPROJECT_CONSTANTS_H
#define CODEBALLPROJECT_CONSTANTS_H
const double EPS = 1e-5;

const double ROBOT_MIN_RADIUS = 1.0;
const double ROBOT_MAX_RADIUS = 1.05;
const double ROBOT_MAX_JUMP_SPEED = 15.0;
const double ROBOT_ACCELERATION = 100.0;
const double ROBOT_NITRO_ACCELERATION = 30.0;
const double ROBOT_MAX_GROUND_SPEED = 30.0;
const double ROBOT_ARENA_E = 0.0;
const double ROBOT_RADIUS = 1.0;
const double ROBOT_MASS = 2.0;

const double TICKS_PER_SECOND = 60.0;
const double MICROTICKS_PER_TICK = 100.0;
const double RESET_TICKS = 2.0 * TICKS_PER_SECOND;
const double BALL_ARENA_E = 0.7;
const double BALL_RADIUS = 2.0;
const double BALL_MASS = 1.0;

const double MIN_HIT_E = 0.4;
const double MAX_HIT_E = 0.5;
const double MAX_ENTITY_SPEED = 100.0;

const double MAX_NITRO_AMOUNT = 100.0;
const double START_NITRO_AMOUNT = 50.0;
const double NITRO_POINT_VELOCITY_CHANGE = 0.6;
const double NITRO_PACK_X = 20.0;
const double NITRO_PACK_Y = 1.0;
const double NITRO_PACK_Z = 30.0;

const double NITRO_PACK_RADIUS = 0.5;
const double NITRO_PACK_AMOUNT = 100.0;
const double NITRO_RESPAWN_TICKS = 10.0 * TICKS_PER_SECOND;
const double GRAVITY = 30.0;

#endif //CODEBALLPROJECT_CONSTANTS_H
