# RobotCar

It is a personal project to convert an old radio control car into a prototype autonomous vehicle. 
More information in the project [Wiki page](https://github.com/AngelJMC/RobotCar/wiki).

## Getting Started


- [AMP 2.6 board](https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-apm25-and-26-overview.rst). You can use Arduino Mega 2560.
- [L298 Driver](http://candy-ho.com/Drivers/Guia%20de%20Uso%20L298.pdf), It is a dual H-bridge to control de DC motor.
- [LM2596 regulator] Step-down DC-DC converter module, ~12v LiPo battery to 5v.

Atom with [PlatformIO](https://platformio.org/) plugin. If you prefer, you can use Arduino IDE.


### Prerequisites

You need install the next arduino library:
- [Encoder@>=1.4.1](https://github.com/PaulStoffregen/Encoder)
- [L298N@>=1.1.0](https://github.com/AndreaLombardo/L298N)
- [PID@>=1.2.](https://github.com/br3ttb/Arduino-PID-Library)
  
All other required libraries are included in the repository.




