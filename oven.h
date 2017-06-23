/*
 * Based on the DCMotorServo libraryby julester23 (https://github.com/julester23/DCMotorServo) 
 * 
 * Adaptation by Tapia Favio for the control of furnace or electric heater
 * 
 * 2017
 * 
 */

#ifndef _OVEN_H
#define _OVEN_H

#include <Arduino.h>
#include <PID_v1.h>
#include <thermo.h>
#include <TimerOne.h>

class OvenControl {
public:
  OvenControl(uint8_t pin_thermocouple, uint8_t pin_zero_crossing, uint8_t pin_PWM_output);
  PID   *myPID;
  void  run();
  void  stop();
  void  move(int new_rela_position);
  void  moveTo(int new_position);
  int   getRequestedTemperature();
  int   getActualTemperature();
  int   getPWMOutput();
  bool  getTurnOnHeater();
  double    getPIDOutput();
  float getCalibrateTC();
  bool  finished();
  bool  setPWMSkip(uint8_t);
  void  setAccuracy(unsigned int);
  void  setTurnOnHeater(bool newValue);
  void  setCalibrateTC(float new_calibrate_val);
  
  //void setCurrentPosition(int);
private:
  uint8_t   _pin_thermocouple;      //Temperature sensor input
  uint8_t   _pin_PWM_output;        //Output for ac voltage control
  uint8_t   _pin_zero_crossing;     //Input for zero crossing detector
  uint8_t   _PWM_output;
  double    _PID_setpoint, _PID_input, _PID_output;
  bool      _turn_on_heater;
  volatile bool _zc;
  volatile uint16_t _dimtime;
  volatile uint8_t _cont;

  kThermocouple *_myTC;
  uint8_t   _pwm_skip;              //The range of PWM to skip
  uint8_t   _temperature_accuracy;  //Set to the highest tolerable inaccuracy (units are encoder counts)
  void      _turnOnHeater();
  friend void zeroCrossInterrupt();
  friend void dim_check();
};


#endif
