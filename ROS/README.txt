Names : 
Groupe ISI 2
Yang David 3673074
Gregoire Kubler 3671219

List of folders :
launch : contain two launchs for gazebo and rviz with turtlebot3

	gazebo.launch spawn our turtlebot in empty world and teleop node
	rviz.launch load the description of our robot and the joint/robot statement with the gui

	In the challenge1: 3 launchs for each tasks
	
	In the challenge2: 3 launchs for each tasks

	In the challenge3: 2 launchs for task1 and task2

rviz : Specificated configuration for rviz

scripts : contain python scripts nodes

	In the challenge1: 3 scripts
		challenge1_task1.py: Following the line thank to the calcul of centroid with a constant speed and stop at the end.
				     Using a P controller for angular speed.
		challenge1_task2.py: Following the line and ajusting the speed if the line is red depending of the color of the centroid and stop also at the end
		challenge1_task3.py: 
	
	In the challenge2: 5 scripts
		challenge1_task1.py: Robot is moving back because of the spawing orientation and stop if detecting obstacle 

		challenge1_task2.py: Robot is servoing to a given distance from parameter server and ajusting the velocity thank to a PID controlling the linear speed
		challenge2_task2_world_control.py: Controlling the movement of the wall 

		challenge1_task3.py: Robot is rotating first to find the correct orientation with a PID controlling the angular speed and 
				     then like task2 servoing the distance to the wall (Not very perpendicular)
		challenge2_task3_world_control.py: Spawing a wall at a random orientation and moving it.

	In the challenge3: 4 scripts
		challenge3_task1.py: Following the line with a simple Proportionnal controlling the angular speed and stop at any obstacles 
				     in front with a specific distance.For the second obstacle send a "True" to a topic to open the door.

		challenge3_task2.py: Same role than the first task but is able to switch between laser data and camera data if is in a labyrinth or not.
				     And able to adapt the speed depending on the line color.

		challenge3a_world_control.py: Controlling the first wall.
		challenge3b_wordl_control.py: Controlling the open door.

urdf : contain the robot description and all xacro extensions,plugins,materials

worlds : contain several maps

src : empty

