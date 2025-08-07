# Micromouse-Project

This project involves the software for a Micromouse designed with an Arduino Uno. The Micromouse is equipped with various Lidars, sensors and motors which we utilise to navigate the Micromouse around the maze. The mazes are photographed from a top-down view beforehand, and then image-processing methods are applied to map out the maze walls. 

From there, we place nodes at each grid of the maze and use Dijkstra's algorithm to map out the shortest path to the exit. Then, we convert the path to instructiosn and send it to our Micromosue to complete the maze. 

The path planning code can be found in the Micromouse branch in the 'micromouse_path_planning.ipynb' file. 

Key Branches: 
* SIMPLEDRIVING -> main robot logic
