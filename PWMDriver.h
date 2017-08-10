/**
 * @file        PWMDriver.h
 *
 * @author      David Zemon
 * @project     MiniSegway
 *
 * @copyright
 * The MIT License (MIT)<br>
 * <br>Copyright (c) 2013 David Zemon<br>
 * <br>Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:<br>
 * <br>The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.<br>
 * <br>THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <PropWare/gpio/dualpwm.h>
#include "globals.h"

using PropWare::DualPWM;
using PropWare::Pin;
using PropWare::Runnable;

class PWMDriver {
    public:
        static const Pin::Mask    MOTOR_1_MASK  = Pin::P10;
        static const Pin::Mask    MOTOR_2_MASK  = Pin::P11;
        static const unsigned int PWM_FREQUENCY = 20000;

    public:
        static int8_t trigger () {
            static DualPWM pwmDriver(PWM_FREQUENCY, MOTOR_1_MASK, MOTOR_2_MASK, &g_leftDuty, &g_rightDuty,
                                     PWM_DRIVER_STACK, PWM_DRIVER_STACK_SIZE);
            return Runnable::invoke(pwmDriver);
        }
};

