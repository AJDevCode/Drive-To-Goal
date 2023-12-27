# Drive to Goal Robot Control


## Description
The `drive-to-goal.py` script is designed to control a robot in a Gazebo simulation. The goal of the script is to drive the robot towards a specified goal point in the simulation, assuming there are no obstacles in the path.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Technologies Used](#technologies-used)


## Prerequisites
Before you begin, ensure you have the following requirements:

- **Robot Simulation Setup**: Set up a robot simulation environment using Gazebo.

- **Python**: Make sure you have Python installed on your system. You can download it from [python.org](https://www.python.org/).

- **Gazebo Simulator**: Install and configure the Gazebo simulator on your system.

- **Linux**: This runs on linux natively or via virtual machine.
 
- **Ros2 Humble**: Need to have this otherwise the program will not run.
  
## Installation
Follow these steps to set up the `drive-to-goal.py` script:


### Step 1: Clone the Repository
```bash
git clone https://github.com/AJDevCode/drive-to-goal-robot.git
```
### Step 2: Install Dependencies
Ros2 Humble needs to be downloaded on linux platform via natively or virtual machine.

### Step 3: Configure Gazebo Environment
Ensure that Gazebo is running and the robot simulation environment is set up with the necessary plugins.

## Usage
To run the drive-to-goal.py script, execute the following command:
``` terminal
python3 drive-to-goal.py
```

The script will control the robot in the Gazebo simulation, driving it towards the specified goal point assuming there are no obstacles.

## Features
The drive-to-goal.py script offers the following key features:

Control a robot in a Gazebo simulation using Python.

Navigate the robot towards a specified goal point.

Assumes a clear path with no obstacles.

## Technologies Used
Python
Gazebo Simulator
Ros2 Humble
