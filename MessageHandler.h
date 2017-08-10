/**
 * @file        MessageReceiver.h
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
#include <PropWare/serial/i2c/i2cmaster.h>
#include "globals.h"

using PropWare::Runnable;
using PropWare::I2CMaster;

class MessageHandler: public Runnable {
    public:
        static int8_t trigger () {
            static MessageHandler messageHandler;
            return Runnable::invoke(messageHandler);
        }

    private:
        static const uint8_t EEPROM_ADDRESS = 0x50 << 1;

    public:
        MessageHandler ()
                : Runnable(MESSAGE_HANDLER_STACK, MESSAGE_HANDLER_STACK_SIZE) {
        }

        virtual void run () {
            const I2CMaster eepromBus;
            const uint16_t  trimAddress = static_cast<uint16_t>(reinterpret_cast<uint32_t>(&g_trim));

            while (1) {
                while (!g_messageReceived);

                const char *message = (*const_cast<JsonObject *>(g_jsonObject))["message"];

                if (0 == strcmp(message, "trim")) {
                    const bool trimAdjustment = (*const_cast<JsonObject *>(g_jsonObject))["value"];
                    g_trim += trimAdjustment ? 0.05 : -0.05;
                    eepromBus.put(EEPROM_ADDRESS, trimAddress, (uint8_t *) &g_trim, sizeof(g_trim));
                } else if (0 == strcmp(message, "move")) {
                    const unsigned int direction = (*const_cast<JsonObject *>(g_jsonObject))["direction"];
                    const unsigned int magnitude = (*const_cast<JsonObject *>(g_jsonObject))["magnitude"];

#ifdef __PROPELLER_32BIT_DOUBLES__
#define sin sinf
#define cos cosf
#endif
                    g_idealAngle = magnitude * sin(direction) / 100;
                    g_turn       = magnitude * cos(direction) / 100;
#ifdef __PROPELLER_32BIT_DOUBLES__
#undef sin
#undef cos
#endif
                }
            }
        }
};
