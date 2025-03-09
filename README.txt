# Multiple_Turtles_Motion
Multiple robots execute different motion planning strategies.

For this project, I developed a system in ROS where three turtles (turtle1, turtle2, and turtle3) are located within a single ROS TurtleSim window. Each turtle follows a specific motion pattern, and they communicate by sending random numbers that influence turtle3's behavior.

Turtle Setup: I created a launch file that specifies the initial positions of all three turtles when the simulation starts.

Motion Planning: The turtles' motion strategies are as follows:

Turtle1 (T1): It draws concentric circles, with the number of half circles specified as an argument. Once it completes its motion, it moves to turtle2's location and sends a random number (n1) between 0 and 1 to turtle2 as a string message.

Turtle2 (T2): After receiving n1 from turtle1, turtle2 moves toward turtle3. Once it reaches turtle3's location, it generates its own random number (n2), concatenates it with n1, and sends the concatenated string (n1 + n2) to turtle3.

Turtle3 (T3): When turtle3 receives the concatenated message from turtle2, it generates its own random number (n3), concatenates it with n1 and n2, forming the string (n1 + n2 + n3). This final string, denoted as "s," determines the movement pattern of turtle3.

Movement Control: The final concatenated string "s" determines the direction of turtle3's movement in a round. Each character in the string corresponds to a direction: "0" means moving right, and "1" means moving left. The round consists of the turtle first moving down along the y-axis and then turning either right or left along the x-axis. For example, if the final string is "010":

Round 1: Turtle3 moves 1 unit down, then turns right and moves 2 units to the right.
Round 2: Turtle3 moves 1 unit down, then turns left and moves 2 units to the left.
Round 3: Turtle3 moves 1 unit down, then turns right and moves 2 units to the right.
Orientation: Throughout the process, the turtles always face the direction in which they are moving.

Implementation: The code was implemented in Python using ROS, and it was tested in the ROS TurtleSim environment. For the motion planning algorithm of each turtle, I created separate Python files. Additionally, I created a launch file to start the simulation with three turtles placed at positions as shown in Figure-1. It’s up to you whether to start the Python nodes within the launch file or run them separately using the "rosrun" command.

By achieving this setup, I successfully demonstrated how ROS can be used to coordinate multiple robots, exchange data, and control their motion based on real-time inputs. This project helped me further understand motion planning, inter-robot communication, and how to structure a multi-turtle system in ROS.
