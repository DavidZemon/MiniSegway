/**
 * @file        SdLogger.h
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

using PropWare::SPI;
using PropWare::SD;
using PropWare::FatFS;
using PropWare::FatFileWriter;
using PropWare::Printer;
using PropWare::Runnable;
using PropWare::CharQueue;

extern const size_t       SD_LOGGER_STACK_SIZE;
extern uint32_t           SD_LOGGER_STACK[];
extern const unsigned int SD_CARD_ERROR_COLOR;

extern volatile unsigned int g_hardFault;
extern volatile double       g_accelValue;
extern volatile double       g_gyroValue;
extern volatile unsigned int g_sensorReaderTimer;
extern volatile double       g_angle;
extern volatile double       g_accelAngle;
extern volatile double       g_gyroAngle;
extern volatile unsigned int g_angleComputerTimer;

class SdLogger: public Runnable {
    public:
        static int8_t trigger (CharQueue &queue) {
            static SdLogger instance(queue);
            return Runnable::invoke(instance);
        }

        SdLogger (CharQueue &queue)
                : Runnable(SD_LOGGER_STACK, SD_LOGGER_STACK_SIZE),
                  m_queue(&queue) {
        }

        virtual void run () {
            PropWare::ErrorCode err;

            SPI spi;
            spi.set_clock(900000);
            const SD      sd(spi);
            FatFS         filesystem(sd);
            FatFileWriter file(filesystem, "SEGWAY.CSV");

            err = do_log(filesystem, file);
            if (!g_hardFault && err) {
                g_hardFault = SD_CARD_ERROR_COLOR;
            }
        }
    private:
        PropWare::ErrorCode do_log (FatFS &filesystem, FatFileWriter &file) {
            PropWare::ErrorCode err;

            check_errors(filesystem.mount());
            if (file.exists())
                check_errors(file.remove());
            check_errors(file.open());
            err = debug(file);

            if (err) {
                file.close();
                filesystem.unmount();
            } else {
                err = file.close();
                if (err) {
                    filesystem.unmount();
                } else {
                    err = filesystem.unmount();
                }
            }

            return err;
        }

        PropWare::ErrorCode debug (FatFileWriter &file) {
            PropWare::ErrorCode err;
            while (!g_hardFault) {
                while (!this->m_queue->is_empty()) {
                    check_errors(file.safe_put_char(m_queue->dequeue()));
                }
            }
            return 0;
        }

    private:
        CharQueue *m_queue;
};
