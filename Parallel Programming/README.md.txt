CMP3103M William Hyde

Summary:
Reactive maze solution which uses the camera image and laser sensors subscribers.
The sensors are used to detect walls around the robot and reacts accordingly depending
on the region of the sensor. The front sensor region and centre laser is used to invoke 
a random left or right turn when within a certain range. The back sensors are used to stop 
the robot from crashing into the walls. The camera image is used to detect colours in the 
maze and react accordingly. The robot follows yellow lines, turns away from red tiles and 
is attracted to the green goal tile.

System start:
- Make sure linux is up to date using the termial and these commands:
	sudo apt-get update
	sudo apt-get upgrade
- Run desired maze in gazebo in the terminal(e.g roslaunch uol_turtlebot_simulator maze1.launch)
- Open MazeNavigation.py in visual studio code
- Run MazeNavigation.py in visual studio code
- Watch robot naviagte maze