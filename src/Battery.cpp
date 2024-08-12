#include "Battery.h"

Battery::Battery(unsigned voltageReadPin, unsigned closeCircuitPin, unsigned lowBatteryVoltage, unsigned R1Value, unsigned R2Value) : 
                    voltageReadPin(voltageReadPin), 
                    closeCircuitPin(closeCircuitPin),
                    lowBatteryVoltage(lowBatteryVoltage),
                    R1Value(R1Value),
                    R2Value(R2Value) {}

void Battery::init() {
    pinMode(voltageReadPin, INPUT);
    pinMode(closeCircuitPin, OUTPUT);
    setCircuitState(true);
}


float Battery::voltageRead() {
    unsigned readValue = analogRead(voltageReadPin);
    float mesuredVoltage = (readValue*5)/1023.0;
    float realVoltage = mesuredVoltage*(R1Value+R2Value)/float(R1Value);
    return realVoltage;
}

void Battery::setCircuitState(bool state) {
    digitalWrite(closeCircuitPin, state);
}

float Battery::manageBattery() {
    float realVoltage = voltageRead();
    if(realVoltage< lowBatteryVoltage) {
        setCircuitState(false);
    }
    else if (realVoltage > lowBatteryVoltage+0.4)
    {
        setCircuitState(true);
    }
    return realVoltage;    
}