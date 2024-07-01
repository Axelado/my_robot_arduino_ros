#include <Arduino.h>

class Motor
{
private:
    unsigned directionControlPin1;
    unsigned directionControlPin2;
    unsigned speedControlPin;       //use a pwm pin

public:
    Motor() = default;
    Motor(unsigned directionControlPin1, 
        unsigned directionControlPin2, 
        unsigned speedControlPin);
    void init(); 
    void moveForward(unsigned pwmSpeed);
    void moveBackward(unsigned pwmSpeed);
    void stop();
};