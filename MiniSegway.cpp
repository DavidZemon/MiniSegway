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

#include "PWMDriver.h"
#include "MessageReceiver.h"
#include "SensorReader.h"
#include "MessageHandler.h"

#include <PropWare/hmi/output/ws2812.h>

using PropWare::WS2812;

const Pin::Mask LEFT_MOTOR_DIRECTION_MASK  = Pin::P12;
const Pin::Mask LEFT_MOTOR_PWM_MASK        = Pin::P13;
const Pin::Mask RIGHT_MOTOR_DIRECTION_MASK = Pin::P14;
const Pin::Mask RIGHT_MOTOR_PWM_MASK       = Pin::P15;

const size_t SENSOR_READER_STACK_SIZE    = 384;
const size_t PWM_DRIVER_STACK_SIZE       = 48;
const size_t MESSAGE_RECEIVER_STACK_SIZE = 64;
const size_t MESSAGE_HANDLER_STACK_SIZE  = 128;

uint32_t SENSOR_READER_STACK[SENSOR_READER_STACK_SIZE];
uint32_t PWM_DRIVER_STACK[PWM_DRIVER_STACK_SIZE];
uint32_t MESSAGE_RECEIVER_STACK[MESSAGE_RECEIVER_STACK_SIZE];
uint32_t MESSAGE_HANDLER_STACK[MESSAGE_HANDLER_STACK_SIZE];
uint8_t  I2C_INTERNAL_BUFFER[I2C_BUFFER_SIZE];

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

const size_t CONSOLE_LOGGER_STACK_SIZE = 196;
uint32_t     CONSOLE_LOGGER_STACK[CONSOLE_LOGGER_STACK_SIZE];
using PropWare::UARTTX;
static UARTTX g_serialBus;
#elif LOG_CONSOLE == LOG_CONSOLE_SHORT
const unsigned int LOG_FREQUENCY = 100;
#endif

volatile unsigned int g_hardFault         = 0;
volatile double       g_angle             = 0;
volatile double       g_accelAngle;
volatile double       g_gyroAngle;
volatile bool         g_stabilizationWait = true;
volatile double       g_accelValueAcosAxis;
volatile double       g_accelValueAsinAxis;
volatile double       g_gyroValue;
volatile unsigned int g_sensorReaderTimer;
volatile JsonObject   *g_jsonObject;
volatile bool         g_messageReceived   = false;
volatile double       g_idealAngle;
volatile double       g_turn;
volatile double       g_trim              = 0;
volatile unsigned int g_leftDuty          = 0;
volatile unsigned int g_rightDuty         = 0;

volatile double  g_pidError;
volatile double  g_pidIntegral;
volatile double  g_pidDerivative;
volatile int32_t g_pidOutput;

void error_led (const unsigned int color);

int main () {
    const auto sensorReaderCogID    = SensorReader::trigger();
    const auto messageReceiverCogID = MessageReceiver::trigger();
    const auto messageHandlerCogID  = MessageHandler::trigger();

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

    // Wait for the angle to stabilize
    waitcnt(SECOND + CNT);
    g_stabilizationWait = false;
    const auto pwmDriverCogID = PWMDriver::trigger();
    const Pin  motorsEnabled(Port::P17, Pin::Dir::OUT);
    motorsEnabled.set();
#if LOG_CONSOLE == LOG_CONSOLE_SHORT
    const auto period = SECOND / LOG_FREQUENCY;
    auto timer = CNT + period;
    while (!g_hardFault) {
        //pwOut.puts("Angle: ");
        pwOut.put_float(g_gyroValue, 6, 3, '0');
        pwOut.put_char('\n');
        timer = waitcnt2(timer, period);
    }
#else
    while (!g_hardFault)
        waitcnt(MILLISECOND + CNT);
#endif
    motorsEnabled.clear();

    cogstop(sensorReaderCogID);
    cogstop(pwmDriverCogID);
    cogstop(messageReceiverCogID);
    cogstop(messageHandlerCogID);
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
