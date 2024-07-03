#include <Arduino.h>
#include "Motor.h"
#include "commands.h"
#define MAX_PWM 255
#include "encoder_driver.h"

void setMotorSpeeds(int leftSpeed, int rightSpeed);

#include "diff_controller.h"

#define BAUDRATE 57600

/* Run the PID loop at 30 times per second */
#define PID_RATE 30 // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
#define AUTO_STOP_INTERVAL 4000
long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

#define leftBackMotorEN 5
#define leftBackMotorIN1 4
#define leftBackMotorIN2 6

#define rightBackMotorEN 9
#define rightBackMotorIN1 7
#define rightBackMotorIN2 8

#define leftFrontMotorEN 10  // TODO: change
#define rightFrontMotorEN 11 // TODO: change

Motor leftBackMotor(leftBackMotorIN1, leftBackMotorIN2, leftBackMotorEN);
Motor rightBackMotor(rightBackMotorIN1, rightBackMotorIN2, rightBackMotorEN);
Motor leftFrontMotor(leftBackMotorIN1, leftBackMotorIN2, leftFrontMotorEN);
Motor rightFrontMotor(rightBackMotorIN1, rightBackMotorIN2, rightFrontMotorEN);



/* Clear the current command parameters */
void resetCommand()
{
    cmd = NULL;
    memset(argv1, 0, sizeof(argv1));
    memset(argv2, 0, sizeof(argv2));
    arg1 = 0;
    arg2 = 0;
    arg = 0;
    index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand()
{
    int i = 0;
    char *p = argv1;
    char *str;
    int pid_args[4];
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);

    switch (cmd)
    {
    case GET_BAUDRATE:
        Serial.println(BAUDRATE);
        break;
    case ANALOG_READ:
        Serial.println(analogRead(arg1));
        break;
    case DIGITAL_READ:
        Serial.println(digitalRead(arg1));
        break;
    case ANALOG_WRITE:
        analogWrite(arg1, arg2);
        Serial.println("OK");
        break;
    case DIGITAL_WRITE:
        if (arg2 == 0)
            digitalWrite(arg1, LOW);
        else if (arg2 == 1)
            digitalWrite(arg1, HIGH);
        Serial.println("OK");
        break;
    case PIN_MODE:
        if (arg2 == 0)
            pinMode(arg1, INPUT);
        else if (arg2 == 1)
            pinMode(arg1, OUTPUT);
        Serial.println("OK");
        break;

    case READ_ENCODERS:
        Serial.print(readEncoder(LEFT));
        Serial.print(" ");
        Serial.println(readEncoder(RIGHT));
        break;
    case RESET_ENCODERS:
        resetEncoders();
        // resetPID();
        Serial.println("OK");
        break;
    case MOTOR_SPEEDS:
        /* Reset the auto stop timer */
        lastMotorCommand = millis();
        if (arg1 == 0 && arg2 == 0)
        {
            setMotorSpeeds(0, 0);
            // resetPID();
            moving = 0;
        }
        else
            moving = 1;
        leftPID.TargetTicksPerFrame = arg1;
        rightPID.TargetTicksPerFrame = arg2;
        Serial.println("OK");
        break;
    case MOTOR_RAW_PWM:
        /* Reset the auto stop timer */
        lastMotorCommand = millis();
        // resetPID();
        moving = 0; // Sneaky way to temporarily disable the PID
        setMotorSpeeds(arg1, arg2);
        Serial.println("OK");
        break;
    case UPDATE_PID:
        while ((str = strtok_r(p, ":", &p)) != '\0')
        {
            pid_args[i] = atoi(str);
            i++;
        }
        Kp = pid_args[0];
        Kd = pid_args[1];
        Ki = pid_args[2];
        Ko = pid_args[3];
        Serial.println("OK");
        break;
    default:
        Serial.println("Invalid Command");
        break;
    }
}

void setup()
{

    Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
  leftBackMotor.init();
  rightBackMotor.init();
  leftFrontMotor.init();
  rightFrontMotor.init();

  // set as inputs
  DDRD &= ~(1 << LEFT_ENC_PIN_A);
  DDRD &= ~(1 << LEFT_ENC_PIN_B);
  DDRC &= ~(1 << RIGHT_ENC_PIN_A);
  DDRC &= ~(1 << RIGHT_ENC_PIN_B);

  // enable pull up resistors
  PORTD |= (1 << LEFT_ENC_PIN_A);
  PORTD |= (1 << LEFT_ENC_PIN_B);
  PORTC |= (1 << RIGHT_ENC_PIN_A);
  PORTC |= (1 << RIGHT_ENC_PIN_B);

  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
  PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);

}

void loop()
{
    while (Serial.available() > 0)
  {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13)
    {
      if (arg == 1)
        argv1[index] = NULL;
      else if (arg == 2)
        argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ')
    {
      // Step through the arguments
      if (arg == 0)
        arg = 1;
      else if (arg == 1)
      {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else
    {
      if (arg == 0)
      {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2)
      {
        argv2[index] = chr;
        index++;
      }
    }
  }


// If we are using base control, run a PID calculation at the appropriate intervals
  if (millis() > nextPID)
  {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    ;
    setMotorSpeeds(0, 0);
    moving = 0;
  }

}

void setMotorSpeed(int i, int spd)
{
    unsigned char reverse = 1;

    if (spd < 0)
    {
        spd = -spd;
        reverse = 0;
    }
    if (spd > 255)
        spd = 255;

    if (i == LEFT)
    {
        if (reverse == 0)
        {
            leftBackMotor.moveForward(spd);
            leftFrontMotor.moveForward(spd);
            // analogWrite(leftFrontMotorEN, spd);
        }
        else if (reverse == 1)
        {
            leftBackMotor.moveBackward(spd);
            leftFrontMotor.moveBackward(spd);
            // analogWrite(leftFrontMotorEN, -spd);
        }
    }
    else /*if (i == RIGHT) //no need for condition*/
    {
        if (reverse == 0)
        {
            rightBackMotor.moveForward(spd);
            rightFrontMotor.moveForward(spd);
            // analogWrite(rightFrontMotorEN, spd);
        }
        else if (reverse == 1)
        {
            rightBackMotor.moveBackward(spd);
            rightFrontMotor.moveBackward(spd);
            // analogWrite(rightFrontMotorEN, -spd);
        }
    }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
}