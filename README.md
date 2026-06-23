# ENEE 467 Fall 2026: Robotics Project Laboratory
## Lab 2 - Part 2 (Hardware): Robot Arm Kinematics on the Real UR3e

This repository contains a Docker container for Lab 2, Part 2 (Hardware) along with the code template needed to complete the procedure and exercises.

## Overview

In this part of the lab you move the forward- and inverse-kinematics tools from Part 1 onto the real UR3e arm. Working only through the Universal Robots Python interface (`ur_rtde`), you connect to the robot over the network, read its live joint angles and reported tool-center-point (TCP) pose, and use that state to validate your kinematic model against the hardware. The work is **read-only**: no motion commands are ever sent to the robot. You will validate forward kinematics by comparing a model-based tool pose against the robot's reported pose, and validate inverse kinematics by recovering a joint configuration from a measured pose. There is no ROS 2 in this part.

## Lab Software

To avoid software conflicts and increase portability, all lab software is packaged as a Docker container. The container already has the Robotics Toolbox for Python and the `ur_rtde` interface installed. Follow the instructions below to get started.

## Building the Docker Image

First check to see if the image is prebuilt on the lab computer by running the following command
```
docker image ls
```
If you see the image named `lab-2-image` in the list then you can **skip** the build process.

To build the Docker image, ensure that you have [Docker](https://www.docker.com/get-started/) installed and the Docker daemon running.
* Clone the hardware branch of this repository and navigate to the `docker` folder
    ```
    cd ~/Labs
    git clone -b hardware https://github.com/ENEE467-F2025/lab-2.git lab-2-hw
    cd lab-2-hw/docker
    ```
* Build the image with Docker compose
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-2-compose.yml build
    ```

## Starting the Container

The lab computers contain a prebuilt image so you will not have to build the image.
* Clone the hardware branch to get the lab-2 code if you haven't done so already
    ```
    cd ~/Labs
    git clone -b hardware https://github.com/ENEE467-F2025/lab-2.git lab-2-hw
    cd lab-2-hw/docker
    ```
* Run the Docker container
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-2-compose.yml run --rm lab-2-docker
    ```
* Once inside the container, you should be greeted with the following prompt indicating that the container is running
    ```
    (lab-2) robot@docker-desktop:~$
    ```
* Edit the lab-2 Python code within `lab-2/src` from a VS Code editor on the host machine. The repo directory `lab-2/src` is mounted into the Docker container at `/home/robot/lab-2/src`, so all changes are reflected **inside** the container.

* With the UR3e powered on and reachable on the network, run the validation script from the terminal running the container:
    ```
    cd ~/lab-2/src
    python ur3_hw_validate.py
    ```
The script connects to the robot, prints its measured joint angles and TCP pose, and reports the forward- and inverse-kinematics validation results. See the lab manual for the exact procedure.

## Lab Instructions

Please follow the [lab manual](Lab_2_Robot_Arm_Kinematics-Part-2.pdf) closely. All instructions are contained inside the lab manual.
