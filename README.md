# Constraint SLAM
This program is a SLAM implementation using constraint programming approach.
It uses the range and bearing observations of identified landmarks with unknown coordinates to improve it's spacial position estimation.
In this scenario, the robot knows its starting coordinates with an uncertainty, and it's velocity informations.

## Inital system state: Dead reckoning estimation
![Dead reckoning position estimation](./images/DeadReck.png)
Here the real robot trajectory is displayed as a red line, and the landmarks as orange boxes.
The blue area can be interpreted as the robots' position uncertainty, when using only the integral of velocity (measured from an INS) and integrating the position. 
This method shows an increasing drift in the position uncertainty.

## Using obstacle range and bearing observations
![Range and bearing positioning](./images/finalTrajectory.png)
In this scenario, the robot is able to detect and identify a random marker at each dt.
The position of the obstacle is unknown, and we only have distance and bearing information, with a margin of error.

Those informations are then fed into a Tubex contractor network as constraints on the robot position, which allow for a better position estimation.

The result is shown as the blue tube, that much better than the initial dead reckoning approach (grey).
The marker positions have also been calculated with a margin of error, and are displayed as black boxes.


## Using real time estimation
![Real time positioning](./images/realTimeProcessing.png)

This time, we make an estimation at each dt. 
The result is a chain of boxes, each containing with certainty the robot position at the instant of the measure.
This result varies from the last one because we cannot constraint the position with the futur observations like we did previously.

This implementation could be integrated in an autonomous robot, to reduce the uncertainty on the current robot position.


A big thanks to Simon Rohou for the course "Constraint Programming for Mobile Robotics" on which this program is based.
Link: http://simon-rohou.fr/research/tubex-lib/doc/index.html
