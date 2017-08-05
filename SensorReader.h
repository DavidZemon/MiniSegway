/**
 * @file        SensorReader.h
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

#include <math.h>

#include <PropWare/concurrent/runnable.h>
#include <PropWare/sensor/gyroscope/l3g.h>
#include <PropWare/sensor/accelerometer/adxl345.h>
#include <PropWare/utility/utility.h>

using PropWare::SPI;
using PropWare::L3G;
using PropWare::ADXL345;
using PropWare::Pin;
using PropWare::Port;
using PropWare::Runnable;
using PropWare::Utility;

class SensorReader: public Runnable {
    public:
        static const Pin::Mask    SCLK             = Pin::Mask::P4;
        static const Pin::Mask    MOSI             = Pin::Mask::P5;
        static const Pin::Mask    MISO             = Pin::Mask::P6;
        static const Pin::Mask    ACCELEROMETER_CS = Pin::Mask::P7;
        static const Pin::Mask    GYRO_CS          = Pin::Mask::P8;
        static const unsigned int SPI_FREQ         = 1000000;

        static const L3G::DPSMode      GYRO_RESOLUTION         = L3G::DPS_250;
        static const ADXL345::Range    ACCELEROMETER_RANGE     = ADXL345::_2G;
        static const ADXL345::DataRate ACCELEROMETER_DATA_RATE = ADXL345::_3200_HZ;
        static const L3G::Axis         GYRO_AXIS               = L3G::Y;
        static const ADXL345::Axis     ACCEL_AXIS_ACOS         = ADXL345::Z;
        static const ADXL345::Axis     ACCEL_AXIS_ASIN         = ADXL345::Y;
        static const unsigned int      GYRO_OFFSET             = 224;

    public:
        static int8_t trigger () {
            static SensorReader instance;
            return Runnable::invoke(instance);
        }

        virtual void run () {
            static const unsigned int updatePeriodInTicks = SECOND / SENSOR_UPDATE_FREQUENCY;

            this->init();

            auto timer = CNT + updatePeriodInTicks;
            while (1) {
                volatile auto fullLoopStart = CNT;

                this->read_accelerometer(g_accelValueAcosAxis, g_accelValueAsinAxis);
                g_gyroValue         = this->read_gyro();
                g_sensorValuesReady = true;

                g_sensorReaderTimer = Utility::measure_time_interval(fullLoopStart);
                timer               = waitcnt2(timer, updatePeriodInTicks);
            }
        }

    private:
        void init () {
            SPI::get_instance().set_mosi(MOSI);
            SPI::get_instance().set_miso(MISO);
            SPI::get_instance().set_sclk(SCLK);
            SPI::get_instance().set_clock(SPI_FREQ);
            SPI::get_instance().set_mode(SPI::Mode::MODE_3);
            SPI::get_instance().set_bit_mode(SPI::BitMode::MSB_FIRST);

            DIRA |= ACCELEROMETER_CS | GYRO_CS; // Set chip selects to output

            this->init_gyro();
            this->init_accelerometer();
        }

        void init_gyro () const {
            this->m_gyro.set_dps(GYRO_RESOLUTION);
            this->m_gyro.write(L3G::Register::CTRL_REG1, 0b11001111); // Data rate = 760 Hz, Low-pass filter = 30 Hz
            this->m_gyro.write(L3G::Register::CTRL_REG5, PropWare::BIT_4); // Enable HP filter
            this->m_gyro.write(L3G::Register::CTRL_REG2, 4); // High-pass filter = 3.5 Hz
        }

        void init_accelerometer () const {
            ADXL345::RateAndPowerMode rateAndPowerMode;
            rateAndPowerMode.fields.dataRate = ACCELEROMETER_DATA_RATE;
            this->m_accelerometer.write(ADXL345::Register::RATE_AND_POWER_MODE, rateAndPowerMode.raw);

            ADXL345::DataFormat dataFormat;
            dataFormat.fields.range = ACCELEROMETER_RANGE;
            this->m_accelerometer.write(ADXL345::Register::DATA_FORMAT, dataFormat.raw);

            this->m_accelerometer.start();
        }

        void read_accelerometer (volatile double &acosAxis, volatile double &asinAxis) const {
            int16_t individualReadings[3];
            this->m_accelerometer.read(individualReadings);
            acosAxis = ADXL345::scale(individualReadings[ACCEL_AXIS_ACOS], ACCELEROMETER_RANGE);
            asinAxis = ADXL345::scale(individualReadings[ACCEL_AXIS_ASIN], ACCELEROMETER_RANGE);
        }

        double read_gyro () const {
            const auto raw = this->m_gyro.read(GYRO_AXIS);
            return L3G::to_dps(raw - GYRO_OFFSET, GYRO_RESOLUTION);
        }

    private:
        SensorReader ()
                : Runnable(SENSOR_READER_STACK, SENSOR_READER_STACK_SIZE),
                  m_accelerometer(ACCELEROMETER_CS, false),
                  m_gyro(SPI::get_instance(), GYRO_CS, false) {
            DIRA &= ~(ACCELEROMETER_CS | GYRO_CS); // Set chip selects back to input
        }

    private:
        const ADXL345 m_accelerometer;
        const L3G     m_gyro;
};
