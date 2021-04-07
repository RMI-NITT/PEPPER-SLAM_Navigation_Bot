#ifndef POLOLU_MC33926_H
#define POLOLU_MC33926_H

#include <Arduino.h>

class MC33926
{
  public:
    // Constructors
    #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(_SAM3XA_)
    // Teensy 3.X and Arduino Due analog pins cannot take 5V
    MC33926(uint8_t DIR1, uint8_t DIR2,uint8_t PWM, uint8_t SF);
    #else
    MC33926(uint8_t DIR1, uint8_t DIR2,uint8_t PWM, uint8_t SF, uint8_t FB);
    #endif
    // Public functions
    void init();  // Initializes the pins
    void set_pwm(int16_t desired_pwm);  // Sets the desired PWM value
    bool fault();  // Checks for an fault and returns true if there is one
    void flip_motor_direction();
    #if !defined(__MK20DX128__) || !defined(__MK20DX256__) || !defined(_SAM3XA_)
    int motor_current();
    #endif

  private:
    // Private pin variables
    uint8_t DIR1_;
    uint8_t DIR2_;
    uint8_t PWM_;
    uint8_t SF_;
    #if !defined(__MK20DX128__) || !defined(__MK20DX256__) || !defined(_SAM3XA_)
    uint8_t FB_;
    #endif
    // Private variables
    int8_t invert_direction_;
};

#endif  // POLOLU_MC33926_H
