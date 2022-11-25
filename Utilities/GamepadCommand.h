/*! @file GamepadCommand.h
 *  @brief The GamepadCommand type containing joystick information
 */
#ifndef MAIN_CPP_GAMEPAD_COMMAND_H
#define MAIN_CPP_GAMEPAD_COMMAND_H
#include "MathTypes.h"

struct GamepadCommand {
    GamepadCommand() {
        zero();
        mode = 0;
    }

    bool left, right, up, down, LB, RB, back, start, a, b, x, y, logitech;
    double leftStickAnalog[2], rightStickAnalog[2];
    double LT, RT;
    unsigned int mode;

    void zero() {
        left = false;
        right = false;
        up = false;
        down = false;
        LB = false;
        RB = false;
        logitech = false;
        back = false;
        start = false;
        a = false;
        b = false;
        x = false;
        y = false;

        LT = 0.0;
        RT = 0.0;
        for (int i = 0; i < 2; ++i) {
            leftStickAnalog[i] = 0;
            rightStickAnalog[i] = 0;
        }
    }
};

struct GamepadCommandShared{
    /*!
    * Construct a gamepad and set to zero.
    */
    GamepadCommandShared(){ zero(); }

    bool down, left, up, right, LB, RB,
            back, start, A, B, X, Y;

    double axisLeftXY[2], axisRightXY[2];
    double LT, RT;

    /*!
    * Set all values to zero
    */
    void zero(){
        down = false;
        left = false;
        up = false;
        right = false;
        LB = false;
        RB = false;
        back = false;
        start = false;
        A = false;
        B = false;
        X = false;
        Y = false;

        LT = 0;
        RT = 0;
        for (int i = 0; i < 2; i++){
            axisLeftXY[i] = 0;
            axisRightXY[i] = 0;
        }
    }

    void init(){
        memset(this, 0, sizeof(GamepadCommand));
    }

    void copy(GamepadCommand gamepad){
        memcpy(this, &gamepad, sizeof(GamepadCommand));
    }
};
#endif // MAIN_CPP_GAMEPAD_COMMAND_H
