/**
 * @file        AngleComputer.h
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
#include <PropWare/sensor/gyroscope/l3g.h>
#include <PropWare/sensor/accelerometer/adxl345.h>
#include <PropWare/utility/utility.h>

using PropWare::Runnable;
using PropWare::Utility;

extern const size_t ANGLE_COMPUTER_STACK_SIZE;
extern uint32_t     ANGLE_COMPUTER_STACK[];

extern const unsigned int SENSOR_UPDATE_FREQUENCY;
extern const unsigned int FLAT_ON_FACE_COLOR;

extern volatile unsigned int g_hardFault;
extern volatile double       g_accelValue;
extern volatile double       g_gyroValue;
extern volatile double       g_angle;
extern volatile double       g_accelAngle;
extern volatile double       g_gyroAngle;
extern volatile unsigned int g_angleComputerTimer;
extern volatile bool         g_sensorValuesReady;

class AngleComputer: public Runnable {
    public:
        static constexpr double MAX_LEAN             = 15.0;
        static constexpr double ACCELEROMETER_WEIGHT = 0.5;
        static constexpr double GYRO_WEIGHT          = 1 - ACCELEROMETER_WEIGHT;
    public:
        static int8_t trigger () {
            static AngleComputer instance;
            return Runnable::invoke(instance);
        }

        virtual void run () {
            while (1) {
                while (!g_sensorValuesReady);
                g_sensorValuesReady = false;

                volatile auto fullLoopStart = CNT;
                // If another thread through an error, stop
                if (g_hardFault)
                    while (1);

                const double leanFromAccelerometer = to_degrees(g_accelValue);
                const double leanFromGyro          = g_angle + g_gyroValue / SENSOR_UPDATE_FREQUENCY;

                g_accelAngle = leanFromAccelerometer;
                g_gyroAngle  = leanFromGyro;
                g_angle      = leanFromAccelerometer * ACCELEROMETER_WEIGHT + leanFromGyro * GYRO_WEIGHT;

                if (fabs(g_angle) > MAX_LEAN)
                    g_hardFault = FLAT_ON_FACE_COLOR;

                g_angleComputerTimer = Utility::measure_time_interval(fullLoopStart);
            }
        }

    private:
        static double to_degrees (double value) {
            value        = fmin(fmax(value, -1), 1);
            double angle = std::asin(value);
            return angle * 180 / M_PI;
        }

    private:
        AngleComputer ()
                : Runnable(ANGLE_COMPUTER_STACK, ANGLE_COMPUTER_STACK_SIZE) {
        }
};
