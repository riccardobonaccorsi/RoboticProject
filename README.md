# RoboticProject

## Authors
@riccardobonaccorsi

Elena Badole

Federico Caporale

Sofia Lorengo

## Description
This is the project for the course of Introduction to robotics of the University of Trento.

The task of this project is to develop the code for a robotic manipulator so that it can move legos megablocks of different classes to some location of the table.
All the blocks must be recognized and classified by a camera.

To run this project you need to locate this directory in the [locosim repository](https://github.com/mfocchi/locosim).

## Installation

1. [Follow the installation process of locosim](https://github.com/mfocchi/locosim)
2. Go to the locosim directory

`cd ~/ros_ws/src/locosim`

3. Create a new catkin directory

`mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/`

`catkin_make`

4. Clone this repository in '~/catkin_ws/src'

`cd ~/catkin_ws/src`

`git clone https://github.com/riccardobonaccorsi/RoboticProject`

5. Build the workspace

`cd ~/catkin_ws ; catkin_make`

6. Source the workspace 

`source devel/setup.bash`

## Usage

### Launch the simulation

Run the python code for the simulation

`python3 -i ~/${your_catkin_ws}/src/locosim/robot_control/lab_exercises/lab_palopoli/ur5_generic.py`

### Launch the vision node

Before launch this node, you must modify the string where the image is saved; the sting is the 'save_image' variable.
You need to replace the 'ricky' with your username.

`rosrun beginner_tutorials listener`

### Launch the motion node

`rosrun ur_kinematic move`