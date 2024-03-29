# Reducing uncertainty in collective perception using self-organized hierarchy

This repository accompanies an article submission; details forthcoming.

This repository contains the lua controllers and loop functions for investigating collective perception in [ARGoS](https://github.com/ilpincy/argos3), using the following approaches:

- **HIER** approach (MNS directory) 
- **STIG** approach (Pheromone directory)
- **MEAN** approach (Voter-Mean directory)
- **VOTE** approach (Voter-Mean directory)

## Configuring and running experiments

The default setups for each project contains 100 objects and 8 ground robots. In order to configure an experiment, please follow the steps below:
 
1. The number of the objects should be specified in `src/boilerplate/boilderplate.argos`. For example, if we want to distribute 
   200 objects, we should set the quantity of objects to 200 in `boilderplate.argos`:

   - In order to change the number of objects that should be placed in the environment, the parameter "quantity"
     in the block of code associated with the object prototype in `boilerplate.argos` should be changed 
     (e.g., line 285 in `MNS/src/boilerplate/boilerplate.argos`, line 202 in `Pheromone/src/boilerplate/boilerplate.argos`).

   - The output files have been defined under the output attribute of the loop function node in `boilerplate.argos` (e.g., line 66 of 
     `MNS/src/boilerplate/boilerplate.argos`).

2. To design a new setup based on a new project, change the environmental properties, the robot and UAV models, the arena, the 
   number of robots, etc, in `boilerplate.argos`.

3. After modifying the `boilerplate.argos` file, the following parameters of the loop funtion `boilerplate_loop_functions.cpp` 
   located in `src/boilerplate/` should also be edited accordingly:

   - `Nb_robots`: the number of robots used in the experiment
   - `Nb_blocks`: the number of objects used in the experiment

   Additionally, for the STIG approach, the number of robots should be modified in `Pheromone/src/boilerplate/structrue_quadcopter.lua` (see line 59).

4. In the HIER apporach, to change the topology or formation shape used by the robots, modify the `structure_quadcopter.lua` according to the comments.


## Outputs of experiments

For the HIER and STIG approaches, there is one output file per each experiment: output.csv, recorded in `src/boilerplate/experiments/`. For the MEAN and VOTE approaches, results for both approaches are collected simultaneously for each experiment and there is one output file per robot: output files are named by the robot's ID, e.g., for the robot with ID 5, the output file is `info_V5.csv` and it contains the results of both MEAN and VOTE approaches for that robot, recorded in `src/boilerplate/Robots-Memories/Results/`. 

All `.csv` are tab delimited. The columns of each `.csv` are as follows:

### HIER approach

- time step
- objects, $\sigma$ (i.e., number of objects counted by robot *r* from the beginning of the experiment)
- opinion at the current time step, $\lambda^{\text{app}}$, where parameter ${k}_t^{max}$ = 1000
- *(not used in the paper)* opinion at the current time step, where parameter ${k}_t^{max}$ = 1200
- *(not used in the paper)* opinion at the current time step, where parameter ${k}_t^{max}$ = 1500
- *(not used in the paper)* mean of all previous values of column 3 (i.e., opinions from every time step, where parameter ${k}_t^{max}$ = 1000)

### STIG approach

- *(not used in the paper)* estimate at the current time step **(one column per robot)**
- opinion at the current time step, $\lambda^{\text{app}}$ **(one column per robot)**
- *(not used in the paper)* mean of the opinions of all robots at the current time step (calculated as the mean of the row of entries from the 2nd group of columns)
- time step

### MEAN and VOTE approaches

- time step
- objects, $\sigma$ (i.e., number of objects counted by robot *r* from the beginning of the experiment)
- opinion at the current time step, $\lambda^{\text{app}}$, using the MEAN approach
- opinion at the current time step, $\lambda^{\text{app}}$, using the VOTE approach
- *(not used in the paper)* mean of all previous values of column 3 (i.e., opinions from every time step, using the MEAN approach)
- *(not used in the paper)* mean of all previous values of column 4 (i.e., opinions from every time step, using the VOTE approach)

In the output file of (only) the robot with the highest ID, there are additional columns:

- *(not used in the paper)* mean of the opinions of all robots from every time step, using the MEAN approach (calculated as the mean of all previous values of column 3 from all `.csv` from this experiment)
- *(not used in the paper)* mean of the opinions of all robots from every time step, using the VOTE approach (calculated as the mean of all previous values of column 4 from all `.csv` from this experiment)
- *(not used in the paper)* mean of all previous values of column 5 from all `.csv` from this experiment
- *(not used in the paper)* mean of all previous values of column 6 from all `.csv` from this experiment



## Compilation and installation

To compile and run each project, follow these steps. 

First, checkout the correct version of ARGoS. The version of ARGoS that has been used for this project 
is "5101304c9f6b776458c822a37a973f598b08700c".
To clone this version of ARGoS:

	git clone https://github.com/ilpincy/argos3
	cd argos3
	git checkout 5101304c9f6b776458c822a37a973f598b08700c

To compile and install this version of ARGoS:

	mkdir build
	cd build
	cmake ../src
	make
	sudo make install

After making sure that the correct version has been installed, compile and install the project.

In the main directory of the approach project (i.e., the directory that contains the `src`), 
make a build directory, then compile and install the project:

	cd "project name"
	mkdir build
	cd build
	cmake ../src
	make
	sudo make install

To run the project, go to `src/boilerplate/` and run ARGoS:

	cd ../src/boilerplate
	argos3 -c boilerplate.argos
