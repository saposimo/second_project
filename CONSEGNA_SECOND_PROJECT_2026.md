# Second Robotics Project 2026 - Trascrizione Consegna

## Pagina 1

SECOND ROBOTICS PROJECT

ROBOTICS

## Pagina 2 - The Problem

Provided data:

- Odometry from the robot
- tfs
- Laser scan data

## Pagina 3 - Data

Format: ROS Bag file

Bag available here:

https://drive.google.com/file/d/1Ik1HoX3EpYHnnP8jxwnrkqgFbIeDM1ss/view?usp=sharing

Data:

- `/odom`: odometry data from the robot (noisy)
- `/ugv/rslidar_points`: pointcloud from lidar
- `/tf`: dynamic tf
- `/tf_static`: static tf, like sensors position

## Pagina 4 - The Project: Task 1 Mapping

- Use the bag to create a map of the environment.
- Use the preferred mapping package.
- Write a launch file that starts:
- all required nodes to perform data conversion
- the mapping node
- rviz with config file to show the map, the lidar and the tf, set global frame to map

## Pagina 5 - The Project: Task 1 Mapping

- To build a map you need tf and laserscan:
- The bag contains a pointcloud topic
- you can use a node, e.g., pointcloud2laserscan to convert the data to laserscan

## Pagina 6 - The Project: Task 2 Navigation

- Setup a realistic simulation of the robot using stage.
- Robot size: AgileX Scout Mini Omnidirectional.
- Robot kinematics: Omnidirectional.
- Setup the navigation stack to receive goals and move the simulated robot avoiding obstacles in the generated map.
- Write a goal-publisher node that reads a sequence of goals from a csv and send them to the robot. A new goal is sent when the robot reach the previous one or it's aborted.
- csv file will be in the folder: `second_project/csv/goals.csv`
- an example csv file is provided

## Pagina 7 - The Project: Task 2 Navigation

- Provide a launch file that starts:
- stage simulation with the robot and the map you build during task 1
- nav2 configured to localize in the provided map and drive autonomously the robot avoiding obstacles
- the controller node that publish the goal after reading them from csv, using action
- csv structure: `x,y,theta`
- rviz configured to visualize the map, the tfs, the particle cloud (if amcl is used), the laser scanner, the paths and the goals

## Pagina 8 - The Project: Task 2 Navigation

- Also provide a map folder with:
- png file of the reconstructed map (mandatory)
- serialized map if slam toolbox is used

## Pagina 9 - Deadlines And Requested Files

- Upload only a `tar.gz` file to webeep (only one team member upload the files).
- Inside the archive:
- `info.txt` file (details next slide)
- folders of the nodes you created (with inside `CMakeLists.txt`, `package.xml`, etc...)
- map folder
- do not upload the entire environment (with build and devel folders)
- do not upload the bag files

## Pagina 10 - Deadlines And Requested Files

File txt must contain only the group names with this structure:

```text
codice persona;name;surname
```

You can add another file called `readme.txt` with additional info. I will not always look for it. But if something goes wrong I'll check for explanations.

## Pagina 11 - Some More Requests

- Name the archive with your codice persona.
- Name the package `second_project`.
- Don't use absolute path.
- The project need to be written using c/c++.

## Pagina 12 - Deadlines And Requested Files

Deadline: 18 June (4 weeks)

Max 3 student for team.

N.B.: If the grading is needed earlier you can submit the project before the deadline. Then write us a mail and specify the need for earlier grading in the message and mail title.

Questions:

- write to me via mail (`simone.mentasti@polimi.it`)
- do not write only to Prof. Matteucci

## Pagina 13 - How To Get Less Points

- The project do not compile -> 0 points
- The project has absolute paths -> 0 points
- The archive has build and devel folder -> -1 points
- The archive has bag files -> -1 points
- The project do not open rviz -> -1 points
- The project do open rviz with wrong config -> -1 points
- Two members upload the project -> 1/2 points
- Three members upload the project -> 1/3 points
