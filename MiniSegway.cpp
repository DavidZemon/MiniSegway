/**
 * @file        MiniSegway.cpp
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

#include "AngleComputer.h"
#include "SensorReader.h"
#ifdef LOG
#include "Logger.h"
#include "SdLogger.h"
#endif

#include <PropWare/hmi/output/ws2812.h>

using PropWare::WS2812;

const size_t ANGLE_COMPUTER_STACK_SIZE = 256;
uint32_t     ANGLE_COMPUTER_STACK[ANGLE_COMPUTER_STACK_SIZE];
const size_t SENSOR_READER_STACK_SIZE  = 160;
uint32_t     SENSOR_READER_STACK[SENSOR_READER_STACK_SIZE];
const size_t LOGGER_STACK_SIZE         = 80;
uint32_t     LOGGER_STACK[LOGGER_STACK_SIZE];
const size_t SD_LOGGER_STACK_SIZE      = 160;
uint32_t     SD_LOGGER_STACK[SD_LOGGER_STACK_SIZE];

const size_t LOG_BUFFER_SIZE = 512;
char         logBuffer[LOG_BUFFER_SIZE];

const unsigned int SENSOR_UPDATE_FREQUENCY = 100;
const unsigned int LOG_FREQUENCY           = 40;

const unsigned int FLAT_ON_FACE_COLOR  = 0x080000;
const unsigned int SD_CARD_ERROR_COLOR = 0x080800;

volatile unsigned int g_hardFault         = 0;
volatile double       g_angle             = 0;
volatile double       g_accelAngle;
volatile double       g_gyroAngle;
volatile bool         g_sensorValuesReady = false;
volatile double       g_accelValueAcosAxis;
volatile double       g_accelValueAsinAxis;
volatile double       g_gyroValue;
volatile unsigned int g_angleComputerTimer;
volatile unsigned int g_sensorReaderTimer;

void error_led (const unsigned int color);

int main () {
    const auto angleComputerCogID = AngleComputer::trigger();
    const auto sensorReaderCogID  = SensorReader::trigger();

#ifdef LOG
    CharQueue persistentLogQueue(logBuffer);
    const auto loggerCogID        = Logger::trigger(persistentLogQueue);
    SdLogger::trigger(persistentLogQueue);
#endif


    auto timer = CNT + SECOND / LOG_FREQUENCY;
    while (!g_hardFault) {
        pwOut.puts("Angle: ");
        pwOut.put_float(g_angle, 5, 3, '0');
        pwOut.put_char('\n');
        timer = waitcnt2(timer, SECOND / LOG_FREQUENCY);
    }

    cogstop(angleComputerCogID);
    cogstop(sensorReaderCogID);
#ifdef LOG
    cogstop(loggerCogID);
    // DO NOT force a shutdown of the SD logger. It monitors g_hardFault and will do a safe shutdown by itself
#endif

    error_led(g_hardFault);
}

void error_led (const unsigned int color) {
    const WS2812 led(Port::P16, WS2812::Type::GRB);
    while (1) {
        led.send(color);
        waitcnt(CNT + CLKFREQ / 10);
        led.send(0);
        waitcnt(CNT + CLKFREQ / 10);
    }
}
