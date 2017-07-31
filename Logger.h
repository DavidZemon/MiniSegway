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

#include <PropWare/memory/sd.h>
#include <PropWare/filesystem/fat/fatfilewriter.h>
#include <PropWare/concurrent/runnable.h>
#include <PropWare/utility/collection/charqueue.h>

using PropWare::PrintCapable;
using PropWare::Printer;
using PropWare::Runnable;
using PropWare::CharQueue;

extern const size_t       LOGGER_STACK_SIZE;
extern uint32_t           LOGGER_STACK[];
extern const size_t       LOG_BUFFER_SIZE;
extern const unsigned int LOG_FREQUENCY;

extern volatile double       g_accelValue;
extern volatile double       g_gyroValue;
extern volatile unsigned int g_sensorReaderTimer;
extern volatile double       g_angle;
extern volatile double       g_accelAngle;
extern volatile double       g_gyroAngle;
extern volatile unsigned int g_angleComputerTimer;

class Logger: public Runnable {
    public:
        static int8_t trigger (CharQueue &queue) {
            static Logger instance(queue);
            return Runnable::invoke(instance);
        }

        Logger (CharQueue &queue)
                : Runnable(LOGGER_STACK, LOGGER_STACK_SIZE),
                  m_queue(&queue),
                  m_printer(queue) {
        }

        virtual void run () {
            m_printer << Printer::Format(5, '0', 10, 3);

            m_printer << "Angle,"
                    "Accelerometer Angle,"
                    "Gyroscope Angle,"
                    "Accelerometer Value,"
                    "Gyroscope Value,"
                    "Angle Timer (us),"
                    "Sensor Timer (us),"
                    "Logger Timer (us)\n";

            volatile auto timer = CNT;
            unsigned int  logTime  = 0;
            while (1) {
                if (m_queue->size() < (LOG_BUFFER_SIZE / 2)) {
                    m_printer << g_angle << ','
                              << g_accelAngle << ','
                              << g_gyroAngle << ','
                              << g_accelValue << ','
                              << g_gyroValue << ','
                              << g_angleComputerTimer << ','
                              << g_sensorReaderTimer << ','
                              << logTime
                              << '\n';
                    logTime = Utility::measure_time_interval(timer);
                    timer = CNT;
                }
            }
        }

    private:
        CharQueue *m_queue;
        Printer   m_printer;
};
