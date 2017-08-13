/**
 * @file        PIDController.h
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

#include <cmath>
#include <PropWare/concurrent/runnable.h>
#include <PropWare/gpio/dualpwm.h>
#include <PropWare/utility/utility.h>

using PropWare::Runnable;
using PropWare::Port;
using PropWare::Utility;
using PropWare::DualPWM;

class PIDController : public Runnable {
    public:
        static const unsigned int MOTOR_DEAD_ZONE = 700;
        static constexpr double   KP              = 200;
        static constexpr double   KI              = 0;
        static constexpr double   KD              = 0;

    public:
        static int8_t trigger() {
            static const PIDController pidController;
            return Runnable::invoke(pidController);
        }

    public:
        PIDController()
                : Runnable(PID_CONTROLLER_STACK, PID_CONTROLLER_STACK_SIZE) {
        }

        virtual void run() {
            const Pin leftMotor(LEFT_MOTOR_DIRECTION_MASK, Pin::Dir::OUT);
            const Pin rightMotor(RIGHT_MOTOR_DIRECTION_MASK, Pin::Dir::OUT);

            while (1) {
                while (!g_sensorValuesReady);
                volatile auto timer = CNT;

#ifdef __PROPELLER_32BIT_DOUBLES__
#define fabs fabsf
#endif
                if ((fabs(g_angle - g_trim)) > MAX_LEAN)
                    g_hardFault = FLAT_ON_FACE_COLOR;
#ifdef __PROPELLER_32BIT_DOUBLES__
#undef fabs
#endif

                g_sensorValuesReady = false;

                auto pidResult = this->pid(g_angle - g_trim);

                if (pidResult > 0) {
                    leftMotor.clear();
                    rightMotor.clear();
                    pidResult += MOTOR_DEAD_ZONE;
                } else {
                    leftMotor.set();
                    rightMotor.set();
                    pidResult -= MOTOR_DEAD_ZONE;
                    pidResult += DualPWM::MAX_DUTY;
                }

                g_leftDuty = g_rightDuty = static_cast<unsigned int>(pidResult);

                g_pidControllerTimer = Utility::measure_time_interval(timer);
            }

        }

    private:
        int32_t pid(const double currentAngle) {
            static double previousAngle = 0;
            static double integral      = 0;

            // Proportional
            const auto error = currentAngle - g_idealAngle;

            // Integral
            integral += error;

            // Derivative
            // TODO: How the heck do I deal with a changing ideal angle?
            const auto derivative = currentAngle - previousAngle;

            previousAngle = currentAngle;

            const int32_t output = static_cast<int32_t>(KP * error + KI * integral + KD * derivative);

            g_pidError      = error;
            g_pidIntegral   = integral;
            g_pidDerivative = derivative;
            g_pidOutput     = output;

            return output;
        }
};
