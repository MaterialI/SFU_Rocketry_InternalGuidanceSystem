# SFU_Rocketry_Recovery_System
## The rev 1.0  
Features: 
### Path construction 
> Constructs a line in small areas 200 X 200 km between the current location of system and the final target  
### Path following 
> Uses PID and distance calculation in spherical coordinates and finds the XTE (Cross Track Error) from the path. After that it adjusts the steering to follow it. 
### Course correction 
> Uses PID and course error to make the steering input and correct the heading.
### State Machine 
> Allows the system to recognize the in which stage of the flight it is and does the necessary actions.
> There are four states <ol>  Ascend <li>Descend </li> <li>Landing </li> <li>On the ground </li> </ol>
## Telemetry recording 
> Performs recording of the data received from sensors. Stores it in a CSV (Comma Separated Value) format.

## Sensor Suite
![alt text](https://github.com/MaterialI/SFU_Rocketry_Recovery_System/edit/main/internalGS/teensy-4.1-cover.jpeg)

