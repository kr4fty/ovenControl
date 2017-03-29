# ovenControl

Based on the [DCMotorServo](https://github.com/julester23/DCMotorServo) library by julester23

OvenControl is a small library for PID temperature control. It can be used in any heater or furnace which is fed by AC voltage.

The principle of operation is very simple. The control temperature is set and then it is checked periodically that it is within the range. Otherwise, the relevant control is made using the [ArduinoPID library](https://github.com/br3ttb/Arduino-PID-Library).

The library is responsible for:

* temperature sensing
* detection of zero crossing of voltage ac
* ac voltage control

all the above to keep the temperature as constant as possible.


## Software

Requires
* [ArduinoPID library](https://github.com/br3ttb/Arduino-PID-Library).
* [K-Type_thermocouple-Library](https://github.com/kr4fty/K-Type_thermocouple-Library).

## Hardware

Using the example, the hardware required for proper use is described below.

The system is composed of four main parts:

* **The microcontroller**: brain in charge of the administration of resources.
* **The temperature sensor**: in charge of constantly measuring the temperature of the oven/heater and in this way can make the feedback towards the control for the respective adjustments. For this function a K-type thermocouple  is used.
* **AC control**: in charge of handling AC, in this case I will use 220Vac/50Hz. For this purpose opto-thyristors are used, which act as a solid state relay.
* **Zero crossing detector**: in charge of activating the thyristors from the moment in which the alternating voltage wave is detected crosses by zero.For this function a K-type thermocouple is used.

![alt tag](https://github.com/kr4fty/ovenControl/blob/master/examples/MyCalentador/circuit.png)

## Practical application
[![Oven Control](https://img.youtube.com/vi/Dz5Lqh4SREI/0.jpg)](https://www.youtube.com/watch?v=Dz5Lqh4SREI "PID Oven Control")
