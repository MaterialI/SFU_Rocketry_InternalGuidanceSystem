# SFU_Rocketry_InternalGuidanceSystem
##The rev 1.0<br />
Features: <br />
###Path construction<br />
> Constructs a line in small areas 200 X 200 km between the current location of system and the final target<br />
###Path following<br />
> Uses PID and distance calculation in spherical coordinates and finds the XTE (Cross Track Error) from the path. After that it adjusts the steering to follow it.<br />
###Course correction<br />
> Uses PID and course error to make the steering input and correct the heading<br />
###State Machine<br />
> Allows the system to recognize the in which stage of the flight it is and does the necessary actions<br />
##Telemetry recording<br />
> TODO<br />
