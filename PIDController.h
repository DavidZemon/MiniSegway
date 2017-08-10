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

#include <PropWare/concurrent/runnable.h>
#include <PropWare/gpio/simpleport.h>
#include <PropWare/utility/utility.h>

using PropWare::Runnable;
using PropWare::SimplePort;
using PropWare::Port;
using PropWare::Utility;

class PIDController: public Runnable {
    public:
        static constexpr double KP = 1024;
        static constexpr double KI = 0;
        static constexpr double KD = 0;

        static const Port::Mask LEFT_MOTOR_DIRECTION_PIN_MASK  = Pin::P12;
        static const Port::Mask RIGHT_MOTOR_DIRECTION_PIN_MASK = Pin::P14;

    public:
        static int8_t trigger () {
            static const PIDController pidController;
            return Runnable::invoke(pidController);
        }

    public:
        PIDController ()
                : Runnable(PID_CONTROLLER_STACK, PID_CONTROLLER_STACK_SIZE) {
        }

        virtual void run () {
            const SimplePort leftMotorDirection(LEFT_MOTOR_DIRECTION_PIN_MASK, 2, Port::Dir::OUT);
            const SimplePort rightMotorDirection(RIGHT_MOTOR_DIRECTION_PIN_MASK, 2, Port::Dir::OUT);

            leftMotorDirection.clear();
            rightMotorDirection.clear();

            while (1) {
                while (!g_newAngleReady);
                volatile auto timer = CNT;

                g_newAngleReady = false;

                auto pidResult = this->pid(g_angle);

                if (pidResult > 0) {
                    leftMotorDirection.write(0b01);
                    rightMotorDirection.write(0b01);
                } else {
                    leftMotorDirection.write(0b10);
                    rightMotorDirection.write(0b10);
                    pidResult *= -1;
                }

                g_leftDuty = g_rightDuty = static_cast<unsigned int>(pidResult);

                g_pidControllerTimer = Utility::measure_time_interval(timer);
            }

        }

    private:
        double pid (const double currentAngle) {
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

            const auto output = KP * error + KI * integral + KD * derivative;

            g_pidError      = error;
            g_pidIntegral   = integral;
            g_pidDerivative = derivative;
            g_pidOutput     = output;

            return output;
        }
};
