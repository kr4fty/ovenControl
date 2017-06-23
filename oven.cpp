/*
 * Based on the DCMotorServo libraryby julester23 (https://github.com/julester23/DCMotorServo) 
 * 
 * Adaptation by Tapia Favio for the control of furnace or electric heater
 * 
 * 2017
 * 
 */

#include <oven.h>

static OvenControl *oven;

void zeroCrossInterrupt()
{
 /*
  *  PWM                                         TRIAC
  * =====                                       =======
  * 
  * digitalWrite(PIN, pwm)                      dimming = 255 - Val
  * 
  *         pwm = Val                                 TRIAC activation time
  *      _____Val___                                               |
  *     /           \                                              |
  *      ___________                                               V___
  *     |           |                                              |   |
  * ____!           |_______________.          ____._______________|   |_______.______
  *     0           Val            255             0           255-Val +10uS  255
  * 
  *                                                                 \____Val___/
  *  
  * 
  * all period ON
  * =============
  * pwm = 255                                   dimming = 0
  * 
  *                                             TODO: With 10 uS is enough for the 
  *                                             triac to be active until the next 
  *                                             crossing by zero
  * 
  *      ___________________________                 ___
  *     |                           |               |   |
  * ____!                           !_____      ____|   |_______________________._____
  *     0                          255              0   10uS                   255
  * 
  * 
  * all period OFF
  * ==============
  * pwm = 0                                     dimming = 255????
  * 
  *                                             WARNING: It does not activate the 
  *                                             output in this cycle but it would 
  *                                             activate the output to the next 
  *                                             cycle and should not
  *                                             
  *                                                                              ___
  *                                                                             |   |
  * ____.___________________________._____      ____.___________________________!   !_
  *     0                          255              0                          255
  * 
  */
  
  // Firing angle calculation : 1 full 50Hz wave =1/50=20ms 
  // Every zerocrossing thus: (50Hz)-> 10ms (1/2 Cycle) 
  // For 60Hz => 8.33ms (10.000/120)
  // 10ms=10000us
  // (10000us - 10us) / 256 = 39 (Approx) For 60Hz =>65  
    
  /*int dimming= 255 - oven->_PWM_output;
  if(oven->_turn_on_heater){
    if(dimming==0){
            digitalWrite(oven->_pin_PWM_output, HIGH);
            delayMicroseconds(10);
            digitalWrite(oven->_pin_PWM_output, LOW);
    }
    else if(dimming==255)
            digitalWrite(oven->_pin_PWM_output, LOW);
            
    else if(0<dimming<255){
            int dimtime = (39*dimming);                 // For 60Hz =>65    
            delayMicroseconds(dimtime);                 // Wait till firing the TRIAC    
            digitalWrite(oven->_pin_PWM_output, HIGH);  // Fire the TRIAC
            delayMicroseconds(10);                      // triac On propogation delay 
                                                        // (for 60Hz use 8.33) Some Triacs need a longer period
            digitalWrite(oven->_pin_PWM_output, LOW);   // No longer trigger the TRIAC (the next zero crossing will swith it off) TRIAC
    }
  }
}*/
  int dimming;
  digitalWrite(oven->_pin_PWM_output, LOW);
  if(oven->_PWM_output>0){
    dimming= 255 - oven->_PWM_output;
    if(dimming==0){
      oven->_zc = true;
      oven->_dimtime = 0;
    }
    else if(dimming==255){
      oven->_zc = false;
    }
    else if(0<dimming<255){
      oven->_zc = true;
      oven->_dimtime = (39*dimming);
    }
  }
}
void dim_check() {
  if(oven->_zc == true) {
    if(oven->_cont>=oven->_dimtime) {
      digitalWrite(oven->_pin_PWM_output, HIGH); // turn on light
      oven->_cont=0;  // reset time step counter
      oven->_zc = false; //reset zero cross detection
    }
    else {
      oven->_cont++; // increment time step counter
    }
  }
}


OvenControl::OvenControl(uint8_t pin_thermocouple, uint8_t pin_zero_crossing, uint8_t pin_PWM_output)
{
  _pin_thermocouple = pin_thermocouple;
  _pin_zero_crossing = pin_zero_crossing; 
  _pin_PWM_output = pin_PWM_output;
  
  oven = this;

  //Direction and PWM output
  pinMode(pin_PWM_output, OUTPUT);
  pinMode(pin_zero_crossing, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(_pin_zero_crossing), zeroCrossInterrupt, FALLING) ;
  Timer1.initialize(39);
  Timer1.attachInterrupt(dim_check, 39);

  _PWM_output = 0;  
  _pwm_skip = 0;
  _temperature_accuracy = 2;
  _turn_on_heater = true;
  _myTC = new kThermocouple(_pin_thermocouple);
  
  _PID_input = _myTC->getTempWithFilter();
  _PID_output = 0;
  _PID_setpoint = _PID_input;

  myPID = new PID(&_PID_input, &_PID_output, &_PID_setpoint,2,5,1, DIRECT);

  myPID->SetSampleTime(50);
  myPID->SetOutputLimits(_pwm_skip-0, 255-_pwm_skip);

  //turn the PID on
  myPID->SetMode(AUTOMATIC);
}

/*void OvenControl::setCurrentPosition(int new_position)
{
  _position->write(new_position);
  _PID_input = _position->read();
}*/

void OvenControl::setAccuracy(unsigned int range)
{
  _temperature_accuracy = range;
}

bool OvenControl::setPWMSkip(uint8_t range)
{
  if ( 0 <= range && range < 255) {
    _pwm_skip = range;
    return 1;
  }
  else
    return 0;
}

void OvenControl::setCalibrateTC(float new_calibrate_val)
{
  _myTC->setCalibrate(new_calibrate_val);
}

void OvenControl::setTurnOnHeater(bool newValue)
{
    _turn_on_heater = newValue;
}

//void OvenControl::SetPIDTunings(double Kp, double Ki, double Kd)
//{
//	myPID->SetTunings(Kp, Ki, Kd);
//}

bool OvenControl::finished()
{
  if (abs(_PID_setpoint - _PID_input) < _temperature_accuracy && _PWM_output == 0)
    return 1;
  return 0;
 
}

void OvenControl::move(int new_rela_position)
{
  //use _PID_setpoint so that we don't introduce errors of _position_accuracy
  _PID_setpoint = _PID_setpoint + new_rela_position;
}

void OvenControl::moveTo(int new_position)
{
  _PID_setpoint = new_position;
}

int OvenControl::getRequestedTemperature()
{
  return _PID_setpoint;
}

int OvenControl::getActualTemperature()
{
  return (uint16_t) _myTC->getTempWithFilter();
}
int OvenControl::getPWMOutput()
{
  return _PWM_output;
}

double OvenControl::getPIDOutput()
{
  return _PID_output;
}

bool OvenControl::getTurnOnHeater()
{
    return _turn_on_heater;
}

float OvenControl::getCalibrateTC()
{
    return _myTC->getCalibrate();
}

void OvenControl::run() {
  _PID_input = _myTC->getTempWithFilter();
  myPID->Compute();
  _PWM_output = abs(_PID_output) + _pwm_skip;
  if (abs(_PID_setpoint - _PID_input) < _temperature_accuracy)
  {
    myPID->SetMode(MANUAL);
    _PID_output = 0;
    _PWM_output = 255;
  }
  else
  {
    myPID->SetMode(AUTOMATIC);
  }

  _turnOnHeater();
}

void OvenControl::stop() {
  myPID->SetMode(MANUAL);
  _PWM_output = 255;
}

void OvenControl::_turnOnHeater() {
  if (_PID_output <= 0)
  {
    _turn_on_heater = false;
  }
  else
  {
    _turn_on_heater = true;
  }
  
}
