# obstacle_avoidance_laserscan

## RWA2 - Reaching the Goal by avoiding the obstacles

The aim of assignment was to plan a sequence of operations to be done by a robot in order to execute a path.

To execute the same, I created one python package(rwa2_Nimbekar) which has one source module.

### 1. To start the environment:

#### a. To start the environment, please extract the folder from zip format. and save it in your home directory and run 'roscore'.
		

#### b. Paste the following commands in your bashrc file  (can access via:= gedit ~/.bashrc):
		
			export TURTLEBOT3_MODEL=burger
			source /opt/ros/melodic/setup.bash
			source ~/rwa2_Nimbekar/devel/setup.bash

### 2. To run the file:
#### a. run source ~/.bashrc three different terminal for ease and paste the follwing in each terminal

			Terminal 1: roslaunch turtlebot3_gazebo turtlebot3_world.launch x_pos:=-2 y_pos:=0

			We use this terminal to spawn the Turtlebot3 at the desire start location. Here the desired start location is (-2, 0)

	     (Optional) Terminal 2: roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch  

			Terminal 3: cd rwa2_Nimebkar/src/
				    rosrun controller simple_controller.py (goal_x, goal_y)
		List of Goal Point:( 1,-2), (-1, 2), ( 2, 0), (-1,-2), ( 1, 2)


This submission was successful in avoiding obstacles but unfortunately reaching the goal became the biggest Obstacle.
I plan on working on the code to overcome that obstacle in near future.
		
      
## Instructions to run the code:(In PyCharm)

1. Download the 'rwa2_Nimbekar.zip' folder from Canvas and unzip it.

2. Open Terminal and run roscore

3. Source the enviroment properly and start required environments as instructed above

4. Run:
	cd rwa2_Nimbekar/src/
	rosrun controller simple_controller.py 1 -2

List of Goal Point:( 1,-2), (-1, 2), ( 2, 0), (-1,-2), ( 1, 2)



