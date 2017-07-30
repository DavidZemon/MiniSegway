/**
 * @file    AngleComputer.h
 *
 * @author  David Zemon
 * @project MiniSegway
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

extern const unsigned int    SENSOR_UPDATE_FREQUENCY;

extern volatile bool         g_hardFault;
extern volatile double       g_leanAngle;
extern volatile double       g_accelAngle;
extern volatile double       g_gyroAngle;
extern volatile double       g_accelValue;
extern volatile double       g_gyroValue;
extern volatile unsigned int g_angleComputerTimer;
extern volatile bool         g_sensorValuesReady;

class AngleComputer: public Runnable {
    public:
        static constexpr double MAX_LEAN             = 5.0;
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
                    while(1);

                const double leanFromAccelerometer = to_degrees(g_accelValue);
                const double leanFromGyro          = g_leanAngle + g_gyroValue / SENSOR_UPDATE_FREQUENCY;

                g_accelAngle = leanFromAccelerometer;
                g_gyroAngle  = leanFromGyro;
                g_leanAngle  = leanFromAccelerometer * ACCELEROMETER_WEIGHT + leanFromGyro * GYRO_WEIGHT;

                /*if (fabs(g_leanAngle) > MAX_LEAN)
                    //safety shutoff if the bot falls over
                    error(FLAT_ON_FACE, NULL);*/

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
