### Robot21C
----------------------------------------------------------------------------
FRC Team 4450 2021 Robot Control program.

This is the Command based 2021 competition robot control program reference implementation created by the Olympia Robotics Federation (FRC Team 4450). 
Operates the robot "Gregory" for FRC game "INFINITE RECHARGE".

----------------------------------------------------------------------------
## Instructions to setup development environment for VS Code
1) Follow the instructions [here](https://wpilib.screenstepslive.com/s/currentCS/m/java) to setup the JDK, Visual Studio Code, the FRC plugins and tools. Do not install the C++ portion. You do not need the FRC Update Suite to compile code.
2) Clone this repository to local folder.
3) Open that folder in Visual Studio Code.
4) Build the project using the build project command from the WPILib commands list.

### If RobotLib gets an update:
1) download the RobotLib.json file from the RobotLib Github repo and drop it into the vendordeps folder inside the project folder. Build the project.
****************************************************************************************************************
Version 21C.3

*   The optical sensors on the turret worked fine in P9 and practice portable. They did not work in the new lab.
    Investigation revealed that the sensors are analog, not digital. They were programmed as digital. Technically
    they should have never worked. Can't explain how they seemed to work prior to the move to new lab. But in the
    new lab they would return false all the time. Only returned true with a bright light on them. Perhaps the light levels in those other spaces were just high enough to trigger. In any case, converted the sensors to
    analog in the code and all is well. Tried the digital photo sensors but they did not work so stayed with analog. Only downside to analog is the limited analog ports on RoboRio.

R. Corn
December 2021

Version 21C.2

*   More post season tinkering and doc updates.

R. Corn
June 2021

Version 21C.1

*   Final version of the code after post season development work.

R. Corn
May 2021

Version 21C.0

*	Developed from Robot20C for compatibility with 2021 Wpilib updates and to integrate Wpilib support for
	desktop simulation.

R. Corn
December 2020
