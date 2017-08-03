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

#include <PropWare/hmi/output/ws2812.h>

using PropWare::WS2812;

const size_t ANGLE_COMPUTER_STACK_SIZE = 256;
uint32_t     ANGLE_COMPUTER_STACK[ANGLE_COMPUTER_STACK_SIZE];
const size_t SENSOR_READER_STACK_SIZE  = 160;
uint32_t     SENSOR_READER_STACK[SENSOR_READER_STACK_SIZE];

const unsigned int SENSOR_UPDATE_FREQUENCY = 100;

const unsigned int FLAT_ON_FACE_COLOR  = 0x080000;
const unsigned int SD_CARD_ERROR_COLOR = 0x080800;

#if defined(LOG_SD)
#include "Logger.h"
#include "SdLogger.h"
const size_t     SD_LOGGER_STACK_SIZE       = 196;
uint32_t         SD_LOGGER_STACK[SD_LOGGER_STACK_SIZE];
const size_t     PERSISTENT_LOG_BUFFER_SIZE = 512;
char             PERSISTENT_LOG_BUFFER[PERSISTENT_LOG_BUFFER_SIZE];
static CharQueue persistentLogQueue(PERSISTENT_LOG_BUFFER);
#endif

#if LOG_CONSOLE == LOG_CONSOLE_LONG
#include "Logger.h"
#include <PropWare/serial/uart/uarttx.h>
const size_t CONSOLE_LOGGER_STACK_SIZE = 128;
uint32_t     CONSOLE_LOGGER_STACK[CONSOLE_LOGGER_STACK_SIZE];
using PropWare::UARTTX;
static UARTTX g_serialBus;
#elif LOG_CONSOLE == LOG_CONSOLE_SHORT
const unsigned int LOG_FREQUENCY = 40;
#endif

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

#if LOG_SD
    SdLogger::trigger(persistentLogQueue);
    auto       printerReady          = [] () -> bool {
        return persistentLogQueue.size() < (PERSISTENT_LOG_BUFFER_SIZE / 2);
    };
    Logger     persistantLogger(SD_LOGGER_STACK, printerReady, persistentLogQueue);
    const auto persistantLoggerCogID = Runnable::invoke(persistantLogger);
#endif

#if LOG_CONSOLE == LOG_CONSOLE_LONG
    const auto   consolePrinterReady    = [] () -> bool { return true; };
    const auto   initFunction           = [] () -> void { DIRA |= g_serialBus.get_tx_mask(); };
    const Logger fullConsoleLogger(CONSOLE_LOGGER_STACK, consolePrinterReady, g_serialBus, initFunction);
    const auto   fullConsoleLoggerCogID = Runnable::invoke(fullConsoleLogger);
    Utility::bit_clear(DIRA, static_cast<PropWare::Bit>(g_serialBus.get_tx_mask())); // Release the TX pin
#endif

#if LOG_CONSOLE == LOG_CONSOLE_SHORT
    auto timer = CNT + SECOND / LOG_FREQUENCY;
    while (!g_hardFault) {
        pwOut.puts("Angle: ");
        pwOut.put_float(g_angle, 5, 3, '0');
        pwOut.put_char('\n');
        timer = waitcnt2(timer, SECOND / LOG_FREQUENCY);
    }
#else
    while (!g_hardFault)
        waitcnt(MILLISECOND + CNT);
#endif

    cogstop(angleComputerCogID);
    cogstop(sensorReaderCogID);
#if LOG_CONSOLE == LOG_CONSOLE_LONG
    cogstop(fullConsoleLoggerCogID);
#endif
#ifdef LOG_SD
    cogstop(persistantLoggerCogID);
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
