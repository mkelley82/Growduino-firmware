#include "GrowduinoFirmware.h"
#include "ec.h"
extern Config config;

uint8_t _echoBit;
volatile uint8_t *_echoInput;
unsigned int _maxEchoTime;
unsigned long _max_time;


void ec_enable() {
#ifdef USE_EC_SENSOR
    pinMode(EC_DATA, INPUT);
    pinMode(EC_ENABLE, OUTPUT);
    _echoBit = digitalPinToBitMask(EC_DATA);       // Get the port register bitmask for the echo pin.
    _echoInput = portInputRegister(digitalPinToPort(EC_DATA));         // Get the input port register for the echo pin.

#ifndef EC_POWER_SAVE
    digitalWrite(EC_ENABLE, HIGH); // power up the sensor
#endif
#endif
}

#define NO_ECHO 0

boolean highStarted() {
    _max_time =  micros() + MAX_SENSOR_DELAY;                  // Set a timeout for the ping to trigger.
    while (*_echoInput & _echoBit && micros() <= _max_time) {} // Wait for echo pin to clear.
    while (!(*_echoInput & _echoBit))                          // Wait for ping to start.
        if (micros() > _max_time) return false;                // Something went wrong, abort.
    _max_time = micros() + _maxEchoTime; // Ping started, set the timeout.
    return true;                         // Ping started successfully.
}

boolean lowStarted() {
    _max_time =  micros() + MAX_SENSOR_DELAY;                  // Set a timeout for the ping to trigger.
    while (!(*_echoInput & _echoBit) && micros() <= _max_time) {} // Wait for echo pin to clear.
    while (*_echoInput & _echoBit)                          // Wait for ping to start.
        if (micros() > _max_time) return false;                // Something went wrong, abort.
    _max_time = micros() + _maxEchoTime; // Ping started, set the timeout.
    return true;                         // Ping started successfully.
}

unsigned int highDuration(){
    if (!highStarted()) return NO_ECHO;                // Trigger a ping, if it returns false, return NO_ECHO to the calling function.
    while (*_echoInput & _echoBit)                      // Wait for the ping echo.
        if (micros() > _max_time) return NO_ECHO;       // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
    return (micros() - (_max_time - _maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
}

unsigned int lowDuration(){
    if (!lowStarted()) return NO_ECHO;
    while (!(*_echoInput & _echoBit))                      // Wait for the ping echo.
        if (micros() > _max_time) return NO_ECHO;       // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
    return (micros() - (_max_time - _maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
}

unsigned long getPulseTime() {
    unsigned long freqhigh = 0;
    unsigned long freqlow =0;
    unsigned long pulseTime=0;
    _maxEchoTime=3000;
    if(highDuration()<2000 && highDuration()>1){
        for(unsigned int j=0; j<EC_SAMPLE_TIMES; j++){
            freqhigh+= highDuration();
            freqlow+= lowDuration();
        }
        pulseTime=((freqhigh + freqlow) / EC_SAMPLE_TIMES);
    } else pulseTime=MINVALUE;
    return pulseTime;
}


int ec_read() {
    int ec = MINVALUE;
#ifdef USE_EC_SENSOR
    long lowPulseTime = 0;
    long highPulseTime = 0;
    long pulseTime;
    float ec_a, ec_b;

    float c_low = 1.278;
    float c_high = 4.523;

    ec_a =  (c_high - c_low) / (config.ec_high_ion - config.ec_low_ion);
    ec_b = c_low - ec_a * config.ec_low_ion;

#ifdef EC_POWER_SAVE
    digitalWrite(EC_ENABLE, HIGH); // power up the sensor
    delay(100);
#endif

   /* 
    for(unsigned int j=0; j<EC_SAMPLE_TIMES; j++){
        highPulseTime+=pulseIn(EC_DATA, HIGH);
        if (j == 0 and highPulseTime == 0)
            return MINVALUE;
        lowPulseTime+=pulseIn(EC_DATA, LOW);
    }
    lowPulseTime = lowPulseTime/EC_SAMPLE_TIMES;
    highPulseTime = highPulseTime/EC_SAMPLE_TIMES;

    pulseTime = (lowPulseTime + highPulseTime)/2;
    */

    pulseTime = getPulseTime();

    ec = (int) 100 * (pulseTime * ec_a + ec_b);

#ifdef DEBUG_CALIB
    Serial.print("echo bit: ");
    Serial.print(_echoBit);
    Serial.print(", echo input: ");
    Serial.print(*_echoInput);
    Serial.print(", EC pulse: ");
    Serial.println(pulseTime);
#endif
#ifndef EC_POWER_SAVE
    digitalWrite(EC_ENABLE, LOW); // power down the sensor
#endif

    if (pulseTime == MINVALUE)
        return MINVALUE;
    return ec;
#else
    return MINVALUE
#endif

}
