/**
 * @file        globals.h
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

#include <ArduinoJson.hpp>
using ArduinoJson::JsonObject;

#define I2C_BUFFER_SIZE 256

extern const size_t ANGLE_COMPUTER_STACK_SIZE;
extern const size_t MESSAGE_RECEIVER_STACK_SIZE;
extern const size_t MESSAGE_HANDLER_STACK_SIZE;

extern uint32_t ANGLE_COMPUTER_STACK[];
extern uint32_t SENSOR_READER_STACK[];
extern uint32_t MESSAGE_RECEIVER_STACK[];
extern uint32_t MESSAGE_HANDLER_STACK[];
extern uint8_t  I2C_INTERNAL_BUFFER[];

extern const unsigned int SENSOR_UPDATE_FREQUENCY;
extern const unsigned int FLAT_ON_FACE_COLOR;

extern const size_t       SENSOR_READER_STACK_SIZE;
extern const unsigned int SENSOR_UPDATE_FREQUENCY;

extern volatile unsigned int            g_hardFault;
extern volatile double                  g_accelAngle;
extern volatile double                  g_gyroAngle;
extern volatile double                  g_angle;
extern volatile double                  g_accelValueAcosAxis;
extern volatile double                  g_accelValueAsinAxis;
extern volatile double                  g_gyroValue;
extern volatile unsigned int            g_sensorReaderTimer;
extern volatile unsigned int            g_angleComputerTimer;
extern volatile bool                    g_sensorValuesReady;
extern volatile ArduinoJson::JsonObject *g_jsonObject;
extern volatile bool                    g_messageReceived;
extern volatile double                  g_idealAngle;
extern volatile double                  g_turn;
extern volatile double                  g_trim;
