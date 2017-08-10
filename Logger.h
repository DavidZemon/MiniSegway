/**
 * @file        Logger.h
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
#include <PropWare/utility/utility.h>

#include "globals.h"

using PropWare::PrintCapable;
using PropWare::Printer;
using PropWare::Runnable;
using PropWare::Utility;

class Logger: public Runnable {
    public:
        static const unsigned int LOG_FREQUENCY = 100;

    public:
        template<size_t N>
        Logger (const uint32_t (&stack)[N], bool (*isPrinterReady) (), PrintCapable &printCapable,
                void(*initFunction) () = NULL)
                : Runnable(stack),
                  m_isPrinterReady(isPrinterReady),
                  m_printer(printCapable),
                  m_initFunction(initFunction) {
        }

        virtual void run () {
            if (this->m_initFunction)
                this->m_initFunction();

            m_printer << Printer::Format(6, '0', 10, 3);

            m_printer << "Angle,"
                    "Accelerometer Angle,"
                    "Gyroscope Angle,"
                    "Accelerometer acos() Value,"
                    "Accelerometer asin() Value,"
                    "Gyroscope Value,"
                    "Angle Timer (us),"
                    "Sensor Timer (us),"
                    "PID Controller Timer (us),"
                    "Logger Timer (us),"
                    "PID Error,"
                    "PID Integral,"
                    "PID Derivative,"
                    "PID Output"
                    "\n";

            volatile auto timer         = CNT;
            unsigned int  logTime       = 0;
            const auto    periodInTicks = SECOND / LOG_FREQUENCY;
            auto          loopTimer     = CNT + periodInTicks;
            while (1) {
                if (this->m_isPrinterReady()) {
                    m_printer << g_angle << ','
                              << g_accelAngle << ','
                              << g_gyroAngle << ','
                              << g_accelValueAcosAxis << ','
                              << g_accelValueAsinAxis << ','
                              << g_gyroValue << ','
                              << g_angleComputerTimer << ','
                              << g_sensorReaderTimer << ','
                              << g_pidControllerTimer << ','
                              << logTime << ','
                              << g_pidError << ','
                              << g_pidIntegral << ','
                              << g_pidDerivative << ','
                              << g_pidOutput
                              << '\n';
                    logTime = Utility::measure_time_interval(timer);
                    timer   = CNT;

                    // Only do a pause if the print invocation didn't take too long
                    if ((logTime + 200) < (1000000 / LOG_FREQUENCY))
                        loopTimer = waitcnt2(loopTimer, periodInTicks);
                    else
                        loopTimer = CNT + periodInTicks;
                }
            }
        }

    private:
        bool (*m_isPrinterReady) ();
        Printer m_printer;
        void (*m_initFunction) ();
};
