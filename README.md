# HOI Readme

## HANDS-ON INTERVENTION: Vehicle-Manipulator Systems

This repository contains the necessary steps to run the HANDS-ON INTERVENTION (HOI) system for Vehicle-Manipulator Systems.

## Prerequisites

Before running the HOI system, make sure you have the following dependencies installed:

- ROS (Robot Operating System)
- Turtlebot
- Python

## Installation

1. Clone the HOI repository to your local machine:

```bash
git clone https://github.com/hassanalhosani/hoi_prol.git
```


## Usage

1. Open a terminal window and launch the Turtlebot simulation:

```bash
roslaunch hoi_prol turtlebot_hoi_basic.launch
```


2. In a different terminal window, run the HOI main script:

```bash
rosrun hoi_prol main.py
```
This will start the HOI system and enable vehicle-manipulator interaction.
