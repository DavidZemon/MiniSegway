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

#include <PropWare/serial/i2c/i2cslave.h>
#include <ArduinoJson.hpp>

#include "globals.h"

using PropWare::I2CSlave;
using PropWare::Runnable;
using ArduinoJson::StaticJsonBuffer;
using ArduinoJson::JsonObject;

class MessageReceiver {
    public:

    private:

    public:
        static int8_t trigger () {
            static I2CSlave<uint8_t> i2CSlave(42, I2C_INTERNAL_BUFFER, I2C_BUFFER_SIZE, MESSAGE_RECEIVER_STACK,
                                              MESSAGE_RECEIVER_STACK_SIZE);

            i2CSlave.set_on_receive([] (I2CSlave<uint8_t> &bus, uint8_t data, size_t remaining) -> void {
                static StaticJsonBuffer<I2C_BUFFER_SIZE> jsonBuffer;
                static char                              buffer[I2C_BUFFER_SIZE];
                static size_t                            index = 0;

                if (index < I2C_BUFFER_SIZE)
                    buffer[index++] = data;

                if (!remaining) {
                    g_jsonObject = &jsonBuffer.parseObject(buffer);

                    if (g_jsonObject != &JsonObject::invalid())
                        g_messageReceived = true;

                    jsonBuffer.clear();
                    index = 0;
                }
            });
            i2CSlave.set_on_request([] (I2CSlave<uint8_t> &bus, uint8_t &data) -> void {});

            return Runnable::invoke(i2CSlave);
        }
};
