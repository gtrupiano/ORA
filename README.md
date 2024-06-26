# Oakland Robotics Association (ORA)

![ORA](Club_Documents/ORA_Logo.png)

## IGVC
Each year, Robotics Association at Oakland University competes in the International Ground Vehicle
Competition (IGVC), specifically the IGVC auto-nav challenge. To compete, a fully autonomous
unmanned ground robotic vehicle needs to be design to travel around an outdoor obstacle course under a
prescribed time while maintaining certain speed constraints. The 2024 IGVC will take place at OU on
5/31/2024-6/3/2024. The Robotics Association will design a completely new robot for this year’s
competition.

### Electrical Subteam Requirements
The project looks to design, implement, and provide documentation for the electrical subsystem of the 2024
International Ground Vehicle Competition (IGVC) intelligent robotic platform. The project team will
interface with both mechanical and software subsystem teams to ensure smooth integration of the developed
electrical subsystem into the main robot. The electrical subsystem will include sensors (such as initial
measurement units, global positioning system, camera, etc.), motors, low-level controllers (such as
Arduino), power distribution system (including batteries, battery management systems, power converters,
etc.), LED display, and safety functions (such as emergency stops, fuses, and light indication). The project
will also involve detailed documentation and a final report that can be integrated into the IGVC final report.

## Components and Layout

### Control Board
![Control_Board](Club_Documents/Control_Board.png)

In the robot, there were plenty of components that needed to interface together. These include both ODrives over CAN and status light control. Its main purpose is to reduce the amount of wires that need to be going from each component and have every necessary component on a single board.


### Robot Control Layout
![Drive_Control](Club_Documents/Drive_Control_Layout.png)
