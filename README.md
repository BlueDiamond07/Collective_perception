# Reducing uncertainty in collective perception using self-organized hierarchy

This repository contains the Lua controllers and loop functions of four approaches to collective perception 
which have been implemented in the ARGoS simulator (https://github.com/ilpincy/argos3).

## Configuring and running experiments

The default setups for each project contains 100 objects and 8 ground robots. In order to run an approach with a set of objects, please follow
the steps below:
 
1- The number of the objects should be specified in src/boilerplate/boilderplate.argos. For example, if we want to distribute 200 objects, we should set the quantity of objects to 200 in the boilderplate.argos:

- In order to change the number of objects/blocks that should be placed in the environment, the parameter "quantity"
  in the block of code associated with the object prototype (which has been specified by comment "blocks" as header; 
  e.g., line 285 in MNS/src/boilerplate/boilerplate.argos, line 202 in Pheromone/src/boilerplate/boilerplate.argos) 
  in the boilerplate.argos should be changed. 

- The output files have been defined under the output attribute of the loop function node (e.g., line 66 of MNS/src/boilerplate/boilerplate.argos). As can be seen in boilerplate.argos, output file(s) have been defined for experiments. For more information on these output files please read section [Output of experiment](#Output-of-experiment).  Furthermore, in addition to the  output files specified in boilerplate.argos, we record a copy of boilerplate.argos file as well.

2- If someone wants to design a new setup based on his/her project, he/she needs to change the environmental properties defined in the boilderplate.argos file. By following the comments in the file, we can change the robot and UAV prototypes, the arena, the number of robots, or whatever is needed for the experiment in the boilerplate.argos file.

3- After modifying the boilerplate.argos file, the loop funtion of the project (boilerplate_loop_functions.cpp), which is located 
   in the src/boilerplate/ directory, should also be edited accordingly. The parameters of the loop function are defined as follows:

- Nb_robots: the number of robots used in the experiment
- Nb_blocks: the number of blocks used in the experiment

By changing the number of blocks or robots in the boilerplate.argos file, the two relevant parameters should be adjusted in
the boilerplate_loop_functions.cpp file. Additionally, as for the Stig approach, the number of robots should be specified in Pheromone/src/boilerplate/structrue_quadcopter.lua (i.e., line 59) which is a helper file to simulate the pheromone reaction of the Stig approach.


4- It is worth noting that in the Hier apporach, in order to change the topology or 
   structure of the MNS, we need to modify the structure_quadcopter.lua. All the files have been sufficiently commented
   such that it is very easy to modify different parts of the code accoring to the goals of research.


## Output of experiment

There is one output file for the Hier and Stig approaches per each experiment, called output.csv. This output file is recorded in src/boilerplate/experiments/ directory. Regarding the Mean and Vote approaches, the results are collected simultaneously per each experiment, with parameter P=0.55 for both. For each experiment, the results related to each robot are recorded in a separate CSV file, distinguishable by the robot's ID with the prefix "info_V"; for example, for the robot with ID 5, the output file is info_V5.csv which contains the results of both Mean and Vote approaches for that robot. These output files are recorded in src/boilerplate/Robots-Memories/Results/ directory. All the CSV files are tab separated values. The values of each file for each approach are defined as follows:

### Hier approach:

- the simulation time step
- the total number of objects (blocks) counted by the MNS-brain from the beginning of the experiment 
- the opinion on object density at each time step with ${k}_t^{max}$ = 1000 time steps
- the opinion on object density at each time step with ${k}_t^{max}$ = 1200 time steps (not used in the paper)
- the opinion on object density at each time step with ${k}_t^{max}$ = 1500 time steps (not used in the paper)
- the mean of opinions on object density from the beginning of the experiment when the maximum time window ${k}_t^{max}$ is 1000 time steps (not used in the paper)

### Stig approach:

- the number of objects (blocks) counted at each time step (within the long range radius $\rho2$) divided by the size of the robot's field of view; there is one column for this output per each robot (not used in the paper)
- the opinion on object density at each time step with ${k}_t^{max}$ = 1000 time steps; there is one column for this output per each robot
- the mean of the opinions of all the robots at each time step
- the simulation time step

### Mean and Vote approaches:

1. the simulation time step
2. the total number of objects (blocks) counted by the robot from the beginning of the experiment
3. the opinion on object density produced based on the Mean decision model at each time step with ${k}_t^{max}$ = 1000 time steps
4. the opinion on object density produced based on the Vote decision model at each time step with ${k}_t^{max}$ = 1000 time steps
5. the mean of opinions on object density produced based on the Mean decision model from the beginning of the experiment when the maximum time window ${k}_t^{max}$ is 1000 time steps (not used in the paper)
6. the mean of opinions on object density produced based on the Vote decision model from the beginning of the experiment when the maximum time window ${k}_t^{max}$ is 1000 time steps (not used in the paper)

It is worth noting that in the output file of the robot with the highest ID, in addition to the outputs (tabs/columns) mentioned above, we record the following output values in separate tabs (columns):

7. the mean of the values of column 3 (i.e., the opinion on object density produced based on the Mean decision model at each time step with ${k}_t^{max}$ = 1000 time steps) for all the robots
8. the mean of the values of column 4 (i.e., the opinion on object density produced based on the Vote decision model at each time step with ${k}_t^{max}$ = 1000 time steps) for all the robots
9. the mean of the values of column 5 (i.e., the mean of opinions on object density produced based on the Mean decision model from the beginning of the experiment when the maximum time window ${k}_t^{max}$ is 1000 time steps) for all the robots (not used in the paper)
10. the mean of the values of column 6 (i.e., the mean of opinions on object density produced based on the Vote decision model from the beginning of the experiment when the maximum time window ${k}_t^{max}$ is 1000 time steps) for all the robots (not used in the paper)



## Compilation and installation

To run each project we need to first compile them. To this end, we first should checkout the right version 
of Argos. Then, we can compile Argos and install it. The version of Argos that has been used for this project 
is "5101304c9f6b776458c822a37a973f598b08700c".


In order to clone the right version of Argos, please follow the instruction below:

	git clone https://github.com/ilpincy/argos3
	cd argos3
	git checkout 5101304c9f6b776458c822a37a973f598b08700c


In order to compile and install the right version of Argos, please follow the instruction below:

	mkdir build
	cd build
	cmake ../src
	make
	sudo make install


After making sure that the correct version has been installed, we should compile and install the project following 
the instruction below:

1- In the main folder of each project, there is a "src" folder. We should make a "build" folder beside this folder. 
   Please follow the steps below to create a "build" folder and compile and install a project:

	cd "project name"
	mkdir build
	cd build
	cmake ../src
	make
	sudo make install

2- To run the project that we installed, we should go to the src/boilerplate/ directory (where all the source codes 
   of the project have been placed), and run Argos:

	cd ../src/boilerplate
	argos3 -c boilerplate.argos
