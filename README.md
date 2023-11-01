# shelbyV2
Dragster robot code v2

The main objective of this competition is to be the fast as possible without exceeding the track limits.

The robot uses a camera to track its position and a hall sensor to measure its velocity. The image processing detects a line and retrieves two main errors: a rotational and displacement error. The rotational is used at a lower speed and the displacement at a higher speed. It uses a conventional PID controller to command the robot's direction.

