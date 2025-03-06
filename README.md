# 3546's 2025 Competition code and documentation

Desmos Tool for calculating reef poses: https://www.desmos.com/calculator/qmgj8rtmrq

The green crosses represent the (x,y) position of the apriltag on the field 

Red dots represent the (x,y) position for a right alignment to the april tag while blue represents the left alignment.

Notice how the equations for the red and blue dots have r + c for the left align? This is because our scoring end effector is on the right side of our robot. If it is on the left side you need to switch the sign for both equations.

r = reef spacing

c = the offset of your scoring end effector from the center of the robot. Ours is about 2 1/2 inches to the right of the center

d = half of the robot width with a spacing offset needed to score

After that we simply use path planner to drive to a pose! :)
