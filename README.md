# STM32-LaneKeeper
A autonomous robotic vehicle with the purpose of lane keeping and eventually stopping in the "parking spot".
It uses 4 color sensors on the bottom of the vehicle and is currently programmed to stay between the 2 red lines (which acts as a lane), and stop at the dark green (which acts as a parking spot).

Programmed in `PlatformIO` using an `STM32`, a custom `PID` control loop for DC motors, and `I2C` communication protocol.

Some very short demos below (click on each one to visit the full video):
[![Link to youtube video](./Images/LaneKeeperGif1.gif)](https://www.youtube.com/watch?v=J9wu3cg9o0I)
[![Link to youtube video](./Images/LaneKeeperGif2.gif)](https://www.youtube.com/watch?v=n7hPzN68Uz4)

## Images
The STM32-LaneKeeper:
![LaneKeeper1](./Images/LaneKeeper1.jpg)

The underside of the robot:
![LaneKeeper3](./Images/LaneKeeper3.jpg)

The demo track I created for this project:
![TrackPic](./Images/TrackPic.jpg)

For more images, please see the [images folder](https://github.com/AshishA26/OmniBot/tree/master/Images) of this repo.

## Demos
2 different demos of the PID control loop in action attempting to match the speed of both wheels to the actual desired speed. The first is when the vehicle was placed on the ground, the second was while the vehicle was in the air.
![MotorsOnGround](./Images/MotorsOnGround.gif)
![MotorsInAir](./Images/MotorsInAir.gif)

## Schematics
- Coming soon!
