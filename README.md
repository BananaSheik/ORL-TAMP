MoveIt 2 Arm with Reinforcement Learning Integration
Overview
This project showcases a robotic arm that utilizes MoveIt 2 for motion planning and control, enhanced with a reinforcement learning (RL) policy for skill development. The repository provides the RL policy and pose generation integration with OpenAI Gym, allowing for effective training and evaluation of arm manipulation tasks.

Contents
gym_rl.py/: Contains the reinforcement learning policy implementation used to train the robotic arm and script to train the RL agent using the specified environment.

valid_poses.txt/: Contains the positions that the robotic arm could select

pose_gen.cpp: Generates target poses based on specified parameters.

hello_moveit.cpp: This code inetgrates the policies and actions made by gym_rl.py and the moveit2 robotic arm so that they both can work seamlessly

README.md: This README file, detailing the project and its contents.

Installation
To set up the MoveIt 2 environment, please refer to the official MoveIt 2 documentation. This repository focuses solely on the RL policy and pose generation components. Ensure that you have MoveIt 2 and its dependencies properly installed before proceeding.
