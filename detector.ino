// Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
// This is a human-readable summary of (and not a substitute for) the license.
// Disclaimer
//
// You are free to:
// Share — copy and redistribute the material in any medium or format
// Adapt — remix, transform, and build upon the material
// The licensor cannot revoke these freedoms as long as you follow the license terms.
//
// Under the following terms:
// Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
// NonCommercial — You may not use the material for commercial purposes.
// ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.
// No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
//
// Notices:
// You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.
// No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
//
// github: https://github.com/gusgonnet
// hackster: https://www.hackster.io/gusgonnet
// project: https://www.hackster.io/gusgonnet/gas-carbon-monoxide-motion-and-water-detection-1f16e3
//
// Free for personal use.
//
// https://creativecommons.org/licenses/by-nc-sa/4.0/

#include "elapsedMillis.h"
#include "AnalogSmooth.h"
#include "FiniteStateMachine.h"

#define APP_NAME "Detector"
#define VERSION "Version 0.01"

/*******************************************************************************
 * changes in version 0.01:
       * Initial version

*******************************************************************************/

/*******************************************************************************
TODOs:
*******************************************************************************/

//enable the user code (our program below) to run in parallel with cloud connectivity code
// source: https://docs.particle.io/reference/firmware/photon/#system-thread
SYSTEM_THREAD(ENABLED);

// Gravity: Analog LPG Gas Sensor (MQ5) For Arduino
// https://www.dfrobot.com/product-684.html
#define USE_SENSOR_MQ5 //pin A0
#define MQ5_SENSOR A0
#ifdef USE_SENSOR_MQ5
AnalogSmooth analogSmoothMQ5 = AnalogSmooth(20); // get 20 samples
float mq5Reading;
double mq5ReadingDouble;
float mq5AlarmThreshold = 2000;
#endif

#define MQ_SENSORS_SAMPLE_INTERVAL 50
elapsedMillis mqSensorsSampleInterval;

// Gravity: Analog Carbon Monoxide Sensor (MQ7) For Arduino
// https://www.dfrobot.com/product-686.html
#define USE_SENSOR_MQ7 //pin A1
#define MQ7_SENSOR A1
#ifdef USE_SENSOR_MQ7
AnalogSmooth analogSmoothMQ7 = AnalogSmooth(20); // get 20 samples
float mq7Reading;
double mq7ReadingDouble;
float mq7AlarmThreshold = 2000;
#endif

#define MQ7_HEATING A4

// Gravity: Digital PIR (Motion) Sensor For Arduino
// https://www.dfrobot.com/product-1140.html
// this sensor can work at 3.3V
// if it detects movement, its output goes HIGH
#define USE_SENSOR_PIR //pin A2
#define PIR_SENSOR A2
unsigned int integratorPirSensor = 0;
int pirSensor = 0;

// DC 12V Water Leak Sensor Detector for Home Security Warning Signal System
// https://www.ebay.ca/itm/DC-12V-Water-Leak-Sensor-Detector-for-Home-Security-Warning-Signal-System/390727209431?hash=item5af92811d7:g:wJMAAOSw2WtbgWzv
#define USE_SENSOR_WATER_LEAK //pin A3
#define WATER_LEAK_SENSOR A3
unsigned int integratorWaterLeakSensor = 0;
int waterLeakSensor = 0;

// FSM declaration
#define STATE_OK "Sensor OK"
#define STATE_ALARM "Sensor Alarm"
#define ALARM_MIN 30000 // min amount of time to stay in alarm before coming back to normal

State mq5OkState = State(mq5OkEnterFunction, mq5OkUpdateFunction, mq5OkExitFunction);
State mq5AlarmState = State(mq5AlarmEnterFunction, mq5AlarmUpdateFunction, mq5AlarmExitFunction);
FSM mq5StateMachine = FSM(mq5OkState);
String mq5State = STATE_OK;

State mq7OkState = State(mq7OkEnterFunction, mq7OkUpdateFunction, mq7OkExitFunction);
State mq7AlarmState = State(mq7AlarmEnterFunction, mq7AlarmUpdateFunction, mq7AlarmExitFunction);
FSM mq7StateMachine = FSM(mq7OkState);
String mq7State = STATE_OK;

// this is to heat properly the MQ7 sensor
State mq7LowHeatState = State(mq7LowHeatEnterFunction, mq7LowHeatUpdateFunction, mq7LowHeatExitFunction);
State mq7HighHeatState = State(mq7HighHeatEnterFunction, mq7HighHeatUpdateFunction, mq7HighHeatExitFunction);
FSM mq7HeatingStateMachine = FSM(mq7HighHeatState);
#define STATE_LOW_HEAT "MQ7 low heat"
#define STATE_HIGH_HEAT "MQ7 high heat"
String mq7HeatingState = STATE_HIGH_HEAT;

State pirOkState = State(pirOkEnterFunction, pirOkUpdateFunction, pirOkExitFunction);
State pirAlarmState = State(pirAlarmEnterFunction, pirAlarmUpdateFunction, pirAlarmExitFunction);
FSM pirStateMachine = FSM(pirOkState);
String pirState = STATE_OK;

State waterLeakOkState = State(waterLeakOkEnterFunction, waterLeakOkUpdateFunction, waterLeakOkExitFunction);
State waterLeakAlarmState = State(waterLeakAlarmEnterFunction, waterLeakAlarmUpdateFunction, waterLeakAlarmExitFunction);
FSM waterLeakStateMachine = FSM(waterLeakOkState);
String waterLeakState = STATE_OK;

State sensorsOkState = State(sensorsOkEnterFunction, sensorsOkUpdateFunction, sensorsOkExitFunction);
State sensorsAlarmState = State(sensorsAlarmEnterFunction, sensorsAlarmUpdateFunction, sensorsAlarmExitFunction);
FSM sensorsStateMachine = FSM(sensorsOkState);
String sensorsState = STATE_OK;

/* The following parameters tune the algorithm to fit the particular
application.  The example numbers are for a case where a computer samples a
mechanical contact 10 times a second and a half-second integration time is
used to remove bounce.  Note: DEBOUNCE_TIME is in seconds and SAMPLE_FREQUENCY
is in Hertz */
// source: http://www.kennethkuhn.com/electronics/debounce.c
Timer debounceInputsTimer(20, debounceInputs);
#define DEBOUNCE_TIME 0.1
#define SAMPLE_FREQUENCY 50
#define MAXIMUM (DEBOUNCE_TIME * SAMPLE_FREQUENCY)

/*******************************************************************************
 * Function Name  : setup
 * Description    : this function runs once at system boot
 *******************************************************************************/
void setup()
{
    Serial.begin();

    /*******************************************************************************
    cloud variables and functions for the ambient temperature
    https://docs.particle.io/reference/firmware/photon/#particle-variable-
    Up to 20 cloud variables may be registered and each variable name is limited to a maximum of 12 characters.
    *******************************************************************************/
    Particle.variable("mq5State", mq5State);
    Particle.variable("mq5Reading", mq5ReadingDouble);

    Particle.variable("mq7State", mq7State);
    Particle.variable("mq7HeatingSt", mq7HeatingState);
    Particle.variable("mq7Reading", mq7ReadingDouble);

    Particle.variable("pirState", pirState);
    Particle.variable("waterLeakSta", waterLeakState);
    Particle.variable("sensorsState", sensorsState);

    debounceInputsTimer.start();

#ifdef USE_SENSOR_MQ5 //pin A0
    pinMode(MQ5_SENSOR, INPUT);
#endif
#ifdef USE_SENSOR_MQ7 //pin A1
    pinMode(MQ7_SENSOR, INPUT);
    pinMode(MQ7_HEATING, OUTPUT); //pin A4
#endif
#ifdef USE_SENSOR_PIR //pin A2
    pinMode(PIR_SENSOR, INPUT);
#endif
#ifdef USE_SENSOR_WATER_LEAK //pin A3
    pinMode(WATER_LEAK_SENSOR, INPUT);
#endif

    // publish startup message with firmware version
    Particle.publish(APP_NAME, VERSION, PRIVATE);
    Serial.println(APP_NAME);
}

/*******************************************************************************
 * Function Name  : loop
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
void loop()
{
    // read the MQ5 and MQ7 analog sensors
    readMQsensors();

    // refresh FSMs
    mq5StateMachine.update();
    mq7StateMachine.update();
    pirStateMachine.update();
    waterLeakStateMachine.update();
    sensorsStateMachine.update();

#ifdef USE_SENSOR_MQ7
    //this ifdef avoids having an output or relay bounce up and down
    mq7HeatingStateMachine.update();
#endif
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 HELPER FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : debounceInputs
 * Description    : debounces the contact sensors, triggered by a timer with similar name
 * Return         : nothing
 * Source: http://www.ganssle.com/debouncing-pt2.htm
 * Source: http://www.kennethkuhn.com/electronics/debounce.c
 *******************************************************************************/
void debounceInputs()
{
#ifdef USE_SENSOR_PIR
    debouncePirSensor();
#endif

#ifdef USE_SENSOR_WATER_LEAK
    debounceWaterSensor();
#endif
}

/*******************************************************************************
 * Function Name  : debouncePirSensor
 * Description    : debounces the pir sensor
 * Return         : nothing
 * Source: http://www.ganssle.com/debouncing-pt2.htm
 * Source: http://www.kennethkuhn.com/electronics/debounce.c
 *******************************************************************************/
void debouncePirSensor()
{

    // Step 1: Update the integrator based on the input signal.  Note that the
    // integrator follows the input, decreasing or increasing towards the limits as
    // determined by the input state (0 or 1).
    if (digitalRead(PIR_SENSOR) == LOW)
    {
        if (integratorPirSensor > 0)
            integratorPirSensor--;
    }
    else if (integratorPirSensor < MAXIMUM)
        integratorPirSensor++;

    // Step 2: Update the output state based on the integrator.  Note that the
    // output will only change states if the integrator has reached a limit, either
    // 0 or MAXIMUM.
    if (integratorPirSensor == 0)
        pirSensor = 0;
    else if (integratorPirSensor >= MAXIMUM)
    {
        pirSensor = 1;
        integratorPirSensor = MAXIMUM; /* defensive code if integrator got corrupted */
    }
}

/*******************************************************************************
 * Function Name  :     debounceWaterSensor
 * Description    : debounces the water leak sensor
 * Return         : nothing
 * Source: http://www.ganssle.com/debouncing-pt2.htm
 * Source: http://www.kennethkuhn.com/electronics/debounce.c
 *******************************************************************************/
void debounceWaterSensor()
{

    // Step 1: Update the integrator based on the input signal.  Note that the
    // integrator follows the input, decreasing or increasing towards the limits as
    // determined by the input state (0 or 1).
    if (digitalRead(WATER_LEAK_SENSOR) == LOW)
    {
        if (integratorWaterLeakSensor > 0)
            integratorWaterLeakSensor--;
    }
    else if (integratorWaterLeakSensor < MAXIMUM)
        integratorWaterLeakSensor++;

    // Step 2: Update the output state based on the integrator.  Note that the
    // output will only change states if the integrator has reached a limit, either
    // 0 or MAXIMUM.
    if (integratorWaterLeakSensor == 0)
        waterLeakSensor = 0;
    else if (integratorWaterLeakSensor >= MAXIMUM)
    {
        waterLeakSensor = 1;
        integratorWaterLeakSensor = MAXIMUM; /* defensive code if integrator got corrupted */
    }
}

/*******************************************************************************
 * Function Name  : readMQsensors
 * Description    : this function reads the mq sensors
 *******************************************************************************/
void readMQsensors()
{

    // is time up? no, then come back later
    if (mqSensorsSampleInterval < MQ_SENSORS_SAMPLE_INTERVAL)
    {
        return;
    }

    //is time up, reset timer
    mqSensorsSampleInterval = 0;

    float analog;

#ifdef USE_SENSOR_MQ5
    analog = analogRead(MQ5_SENSOR);
    // Smoothing with window size 10
    mq5Reading = analogSmoothMQ5.smooth(analog);
    mq5ReadingDouble = double(mq5Reading);
#endif

/*******************************************************************************
According to MQ-7 datasheet, sensor has to run through high- and low-heating cycles in order to get proper measurements. 
During low temperature phase, CO is absorbed on the plate, producing meaningful data. 
During high temperature phase, absorbed CO and other compounds evaporate from the sensor plate, cleaning it for the next measurement.

So in general operation is simple:

1. Apply 5V for 60 seconds, don't use these readings for CO measurement.

2. Apply 1.4V for 90 seconds, use these readings for CO measurement.

3. Go to step 1.
 *******************************************************************************/
#ifdef USE_SENSOR_MQ7
    if (mq7HeatingStateMachine.isInState(mq7LowHeatState))
    {
        analog = analogRead(MQ7_SENSOR);
        // Smoothing with window size 10
        mq7Reading = analogSmoothMQ7.smooth(analog);
        mq7ReadingDouble = double(mq7Reading);
    }
#endif
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 FINITE STATE MACHINE FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 MQ5 sensor
*******************************************************************************/
void mq5OkEnterFunction()
{
    mq5SetState(STATE_OK);
}
void mq5OkUpdateFunction()
{
    if (mq5Reading > mq5AlarmThreshold)
    {
        mq5StateMachine.transitionTo(mq5AlarmState);
    }
}
void mq5OkExitFunction()
{
    Particle.publish("ALARM", "Alarm on MQ5 sensor", PRIVATE);
    sensorsStateMachine.transitionTo(sensorsAlarmState);
}
void mq5AlarmEnterFunction()
{
    mq5SetState(STATE_ALARM);
}
void mq5AlarmUpdateFunction()
{
    // stay here a minimum time
    if (mq5StateMachine.timeInCurrentState() < ALARM_MIN)
    {
        return;
    }
    // sensor went back to normal
    if (mq5Reading < mq5AlarmThreshold)
    {
        mq5StateMachine.transitionTo(mq5OkState);
    }
}
void mq5AlarmExitFunction()
{
    Particle.publish("OK", "MQ5 sensor OK", PRIVATE);
}

/*******************************************************************************
 * Function Name  : mq5SetState
 * Description    : sets the state of an FSM
 * Return         : none
 *******************************************************************************/
void mq5SetState(String newState)
{
    mq5State = newState;
    Particle.publish("FSM", "Mq5 fsm entering " + newState + " state", PRIVATE);
}

/*******************************************************************************
 MQ7 sensor
*******************************************************************************/
void mq7OkEnterFunction()
{
    mq7SetState(STATE_OK);
}
void mq7OkUpdateFunction()
{
    if (mq7Reading > mq7AlarmThreshold)
    {
        mq7StateMachine.transitionTo(mq7AlarmState);
    }
}
void mq7OkExitFunction()
{
    Particle.publish("ALARM", "Alarm on MQ7 sensor", PRIVATE);
    sensorsStateMachine.transitionTo(sensorsAlarmState);
}
void mq7AlarmEnterFunction()
{
    mq7SetState(STATE_ALARM);
}
void mq7AlarmUpdateFunction()
{
    // stay here a minimum time
    if (mq7StateMachine.timeInCurrentState() < ALARM_MIN)
    {
        return;
    }
    // sensor went back to normal
    if (mq7Reading < mq7AlarmThreshold)
    {
        mq7StateMachine.transitionTo(mq7OkState);
    }
}
void mq7AlarmExitFunction()
{
    Particle.publish("OK", "MQ7 sensor OK", PRIVATE);
}

/*******************************************************************************
 * Function Name  : mq7SetState
 * Description    : sets the state of an FSM
 * Return         : none
 *******************************************************************************/
void mq7SetState(String newState)
{
    mq7State = newState;
    Particle.publish("FSM", "Mq7 fsm entering " + newState + " state", PRIVATE);
}

/*******************************************************************************
 MQ7 sensor Heating
*******************************************************************************

According to MQ-7 datasheet, sensor has to run through high- and low-heating cycles in order to get proper measurements. 
During low temperature phase, CO is absorbed on the plate, producing meaningful data. 
During high temperature phase, absorbed CO and other compounds evaporate from the sensor plate, cleaning it for the next measurement.

So in general operation is simple:

1. Apply 5V for 60 seconds, don't use these readings for CO measurement.

2. Apply 1.4V for 90 seconds, use these readings for CO measurement.

3. Go to step 1.

*******************************************************************************/
void mq7HighHeatEnterFunction()
{
    mq7HeatingSetState(STATE_HIGH_HEAT);
    // external relay wired in output A4
    digitalWrite(MQ7_HEATING, HIGH);
}
void mq7HighHeatUpdateFunction()
{
    if (mq7HeatingStateMachine.timeInCurrentState() > 60000)
    {
        mq7HeatingStateMachine.transitionTo(mq7LowHeatState);
    }
}
void mq7HighHeatExitFunction()
{
}

void mq7LowHeatEnterFunction()
{
    mq7HeatingSetState(STATE_LOW_HEAT);
// use onboard relay 2 for the heating of the sensor?
// the alternative is to use an external relay wired in output A4
#ifdef MQ7_USE_ONBOARD_RELAY
    triggerRelay(String(MQ7_USE_ONBOARD_RELAY_NUMBER) + "off");
#else
    digitalWrite(MQ7_HEATING, LOW);
#endif
}
void mq7LowHeatUpdateFunction()
{
    if (mq7HeatingStateMachine.timeInCurrentState() > 90000)
    {
        mq7HeatingStateMachine.transitionTo(mq7HighHeatState);
    }
}
void mq7LowHeatExitFunction()
{
}

/*******************************************************************************
 * Function Name  : mq7HeatingSetState
 * Description    : sets the state of an FSM
 * Return         : none
 *******************************************************************************/
void mq7HeatingSetState(String newState)
{
    mq7HeatingState = newState;
    Particle.publish("FSM", "Mq7 HEATING fsm entering " + newState + " state", PRIVATE);
}

/*******************************************************************************
 PIR sensor
*******************************************************************************/
void pirOkEnterFunction()
{
    pirSetState(STATE_OK);
}
void pirOkUpdateFunction()
{
    if (pirSensor == 1)
    {
        pirStateMachine.transitionTo(pirAlarmState);
    }
}
void pirOkExitFunction()
{
    Particle.publish("ALARM", "Alarm on PIR sensor", PRIVATE);
    sensorsStateMachine.transitionTo(sensorsAlarmState);
}
void pirAlarmEnterFunction()
{
    pirSetState(STATE_ALARM);
}
void pirAlarmUpdateFunction()
{
    // stay here a minimum time
    if (pirStateMachine.timeInCurrentState() < ALARM_MIN)
    {
        return;
    }
    // sensor went back to normal
    if (pirSensor == 0)
    {
        pirStateMachine.transitionTo(pirOkState);
    }
}
void pirAlarmExitFunction()
{
    Particle.publish("OK", "PIR sensor OK", PRIVATE);
}

/*******************************************************************************
 * Function Name  : pirSetState
 * Description    : sets the state of an FSM
 * Return         : none
 *******************************************************************************/
void pirSetState(String newState)
{
    pirState = newState;
    Particle.publish("FSM", "PIR fsm entering " + newState + " state", PRIVATE);
}

/*******************************************************************************
 WATER LEAK sensor
*******************************************************************************/
void waterLeakOkEnterFunction()
{
    waterLeakSetState(STATE_OK);
}
void waterLeakOkUpdateFunction()
{
    if (waterLeakSensor == 1)
    {
        waterLeakStateMachine.transitionTo(waterLeakAlarmState);
    }
}
void waterLeakOkExitFunction()
{
    Particle.publish("ALARM", "Alarm on WATER LEAK sensor", PRIVATE);
    sensorsStateMachine.transitionTo(sensorsAlarmState);
}
void waterLeakAlarmEnterFunction()
{
    waterLeakSetState(STATE_ALARM);
    sensorsStateMachine.transitionTo(sensorsAlarmState);
}
void waterLeakAlarmUpdateFunction()
{
    // stay here a minimum time
    if (waterLeakStateMachine.timeInCurrentState() < ALARM_MIN)
    {
        return;
    }
    // sensor went back to normal
    if (waterLeakSensor == 0)
    {
        waterLeakStateMachine.transitionTo(waterLeakOkState);
    }
}
void waterLeakAlarmExitFunction()
{
    Particle.publish("OK", "WATER LEAK sensor OK", PRIVATE);
}

/*******************************************************************************
 * Function Name  : waterLeakSetState
 * Description    : sets the state of an FSM
 * Return         : none
 *******************************************************************************/
void waterLeakSetState(String newState)
{
    waterLeakState = newState;
    Particle.publish("FSM", "WATER LEAK fsm entering " + newState + " state", PRIVATE);
}

/*******************************************************************************
 sensors state machine
 this FSM goes in alarm mode if any sensor gets triggered
 it controls the siren/buzzer - it will turn on relay 1 for 5 minutes
*******************************************************************************/
void sensorsOkEnterFunction()
{
    sensorsSetState(STATE_OK);
}
void sensorsOkUpdateFunction()
{
}
void sensorsOkExitFunction()
{
}
void sensorsAlarmEnterFunction()
{
    sensorsSetState(STATE_ALARM);
}
void sensorsAlarmUpdateFunction()
{
    // if all sensors went back to normal, then go to normal
    if (mq5StateMachine.isInState(mq5OkState) && mq7StateMachine.isInState(mq7OkState) &&
        pirStateMachine.isInState(pirOkState) && waterLeakStateMachine.isInState(waterLeakOkState))
    {
        sensorsStateMachine.transitionTo(sensorsOkState);
    }
}
void sensorsAlarmExitFunction()
{
}

/*******************************************************************************
 * Function Name  : sensorsSetState
 * Description    : sets the state of an FSM
 * Return         : none
 *******************************************************************************/
void sensorsSetState(String newState)
{
    sensorsState = newState;
    Particle.publish("FSM", "sensors fsm entering " + newState + " state", PRIVATE);
}
