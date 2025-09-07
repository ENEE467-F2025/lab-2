# ENEE 467 Fall 2025: Robotics Project Laboratory
## Lab 2: Manipulator Kinematics with the Robotics Toolbox for Python

This repository contains a Docker container for Lab 2 (Manipulator Kinematics with the Robotics Toolbox for Python) as well as the necessary code templates for completing the exercises.

## Overview

Manipulator kinematics refers to the mathematical study of how a robot arm’s joint motions relate to the spatial position and orientation of its end-effector or tool. Put differently, kinematics provides the mapping between the joint space of a robot and its task space (defined by a Cartesian position and orientation). Manipulator kinematics is commonly categorized into two connected sub-problems, namely: forward and inverse kinematics. While forward kinematics computes the end-effector pose from known joint variables, inverse kinematics solves the more challenging problem of finding joint configurations that achieve a desired end-effector pose. Together, they form the math backbone of higher-level robotic planning and control, and are thus crucial for robotic manipulation. You will explore varied solutions to both problems in this lab.

## Lab Software

To avoid software conflicts and increase portability, all lab software will be packaged as a Docker container. Follow the instructions below to get started.

## Building the Container

First check to see if the image is prebuilt on the lab computer by running the following command
```
docker image ls
```
If you see the image named `lab-2-image` in the list then you can **skip** the build process.

To build the Docker container, ensure that you have [Docker](https://www.docker.com/get-started/) installed and the Docker daemon running.
* Clone this repository and navigate to the `docker` folder
    ```
    cd ~/Labs
    git clone https://github.com/ENEE467-F2025/lab-2.git
    cd lab-2/docker
    ```
* Build the image with Docker compose
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-2-compose.yml build
    ```

## Starting the Container

The lab computers contain a prebuild image so you will not have to build the image.
* Clone this repo to get the lab-2 code if you haven't done so already
    ```
    cd ~/Labs
    git clone https://github.com/ENEE467-F2025/lab-2.git
    cd lab-2/docker
    ```
* Run the Docker container
    ```
    docker compose -f lab-2-compose.yml run --rm lab-2-docker
    ```
* Once inside the container, you should be greeted with the following prompt indicating that the container is running
    ```
    (lab-2) robot@docker-desktop:~$
    ```
* Edit the lab-2 Python code  within the `lab-2/src` and `lab-2/src/exercise` from a VS Code editor on the host machine. The repo directory `lab-2/src`  is mounted to the docker container located at `/home/robot/lab-2/src` so all changes will be reflected **inside** the container.
* Test your setup by executing a Python script within the `lab-2/src` directory:
    ```
    cd ~/lab-2/src
    python 2r_dh.py
    ```

## Lab Instructions

Please follow the [lab manual](Lab_2_Manipulator_Kinematics.pdf) closely. All instructions are contained inside the lab manual.