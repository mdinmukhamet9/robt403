# robt403
Dinmukhamet Murat, 201917526.

This is the README file for the LAB1 report.

The folder LAB1 contains all the packages that were created for the purpose of LAB1.
It also contains all the screenshots with appropriate names in order to see how it works
without a need of actually running each of the nodes.

Exercise 0:
I created a package called murat (my surname), which includes the nodes that send my ID number
with given frequency that can be changed in the code (comments in the code included)


Exercise 1:
"my_package" package was created which includes publisher and subscriber nodes with given codes.
CMake file was changed according to the instructions, and after compiling the workspace 
everything worked as expected.

For the following exercises the same package was used due to the fact that they are corellated.

Exercise 2: 
"turtlebot_controller" package was created, and subscriber node in the src/ folder.
Subscriber node has code from turtle_listener, which reads the position and orientation 
from turtlesim_node.

Exercise 3:
Subscriber node was changed so that callback function will pass new velocity to turtlesim_node. 
Turtle is moving in the circular motion

Exercise 4:
Subscriber node was modified, so it will continue its circular motion, but another turtle 
with name Turtle_Dima will be spawned. Spawn service was written in callback function in 
order to check the output message which can bee seen in terminal "Turtle named ______ 
already exists". It prevents from spawning the turtle with the existing name.

Exercise 5:
New node called "exercise5.cpp" was created in the turtlebot_controller package, in folder
src/. This node does the following actions in steps: 
1. Kills turtle1, which is spawned when turtlesim_node is runned
2. Spawns the new turtle called turtle_dima and teleports it to the bottom-left corner.
After that path of the turtle is cleared, because there is the path when turtle is teleported.
3. Turtle starts moving in sqaure path, stops before clearing its path again.
4. Turtle starts new motion in triangular form and stops.
