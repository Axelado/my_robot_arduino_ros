#include <Arduino.h>

class Battery
{
private:
    unsigned voltageReadPin; // Analag Pin
    unsigned closeCircuitPin; // The relais or transistor pin
    unsigned lowBatteryVoltage; 
    unsigned R1Value; // 
    unsigned R2Value; // 

public:
    Battery(unsigned voltageReadPin, unsigned closeCircuitPin, unsigned lowBatteryVoltage,unsigned R1Value, unsigned R2Value);
    void init();
    float voltageRead();
    void setCircuitState(bool state); // true for closed circuit (connected) and false for open circuit (disconnected)
    float manageBattery(); //
};