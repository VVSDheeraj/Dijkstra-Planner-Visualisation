## Running the Simulation Guide

This section outlines the steps to successfully execute the simulation.

### Prerequisites
First, make sure to install all dependencies listed below:
1. import numpy as np
2. import cv2
3. import heapq
4. import time

### Setting Up Your Simulation
1. **Initiate Start and End Points**: Upon initiation, you will be prompted to specify both a starting and an ending point for the simulation. It is crucial to select these points within permissible ranges to circumvent any obstacles and to facilitate the accurate creation of paths.

2. **Consideration of Map Boundaries**: It's essential to account for the dimensions of the map and maintain clearance from boundaries when choosing your points, ensuring the simulation runs smoothly and avoids any impediments.

3. **Position Examples**: For a clear understanding, refer to the example within the demonstration video where the starting point is at coordinates (6, 6) and the goal is positioned at (1094, 493). Feel free to modify these coordinates based on the requirements of your simulation.


### Monitoring the Simulation
The process will automatically progress towards the goal point. Upon reaching the destination, a simulation video showcasing the path will be generated as output.


### Repository Overview
You'll find all necessary scripts and files for running the simulation within the provided GitHub repository. Ensure you explore all contents to fully understand the simulation setup.