/**
 */

#include "AngleComputer.h"
#include "SensorReader.h"

const size_t ANGLE_COMPUTER_STACK_SIZE = 256;
uint32_t     ANGLE_COMPUTER_STACK[ANGLE_COMPUTER_STACK_SIZE];
const size_t SENSOR_READER_STACK_SIZE  = 160;
uint32_t     SENSOR_READER_STACK[SENSOR_READER_STACK_SIZE];

const unsigned int SENSOR_UPDATE_FREQUENCY = 100;

volatile bool         g_hardFault         = false;
volatile double       g_leanAngle         = 0;
volatile double       g_accelAngle;
volatile double       g_gyroAngle;
volatile bool         g_sensorValuesReady = false;
volatile double       g_accelValue;
volatile double       g_gyroValue;
volatile unsigned int g_angleComputerTimer;
volatile unsigned int g_sensorReaderTimer;

static const int PRINT_LOOP_FREQUENCY = 40;

void print_graph (const int markerIndex);

int main () {
    AngleComputer::trigger();
    SensorReader::trigger();

    auto timer = CNT + SECOND / PRINT_LOOP_FREQUENCY;
    while (1) {
        //print_graph(static_cast<const int>(g_leanAngle * 10));
        pwOut.printf("%f", g_leanAngle);
        //pwOut << ", ";
        //pwOut.printf("Accel: %5.2f, Gyro: %5.2f", g_accelValue, g_gyroValue);
        //pwOut.printf(", Angle Timer: %d us, Sensor Timer: %d us", g_angleComputerTimer, g_sensorReaderTimer);
        pwOut << '\n';
        timer = waitcnt2(timer, SECOND / PRINT_LOOP_FREQUENCY);
    }
}

void print_graph (const int markerIndex) {
    pwOut << '|';
    for (int i = -50; i <= 50; ++i) {
        if (i == markerIndex)
            pwOut << '*';
        else if (i < 0)
            pwOut << ' ';
        else if (i == 0)
            pwOut << '|';
        else
            pwOut << ' ';
    }
    pwOut << '|' << '\n';
}
