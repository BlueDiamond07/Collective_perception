#include "boilerplate_loop_functions.h"
#include <argos3/plugins/robots/prototype/simulator/prototype_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/entities/tag_entity.h>
#include <argos3/plugins/simulator/entities/radio_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/media/radio_medium.h>
#include <argos3/core/utility/plugins/factory.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <cmath>
#include <array>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <string> 
#include <fstream>
#include <exception>
#include <experimental/filesystem>
#include <iostream>
#include <functional>
#include <fstream>
namespace argos {
   /****************************************/
   /****************************************/
      const int Nb_robots = 8;
      const int Nb_blocks = 100;	
      float RobotCoordinate[Nb_robots][2]; // for Nb_robots ground robots
      int evaporation[Nb_blocks+1] = {}; // duration for which a pheromone source is active after it has been placed (i.e., triggered)
      int outFlag = 1; // an implementation flag used for switching between block densities in dynamic setups
      float ListOfBlocksDensities[Nb_robots+Nb_blocks+2][50500]={0}; 
      double opinion[Nb_robots+Nb_blocks+2][50500]={0};
      double Average_AllRobots_Local = 0;
      double Average_AllRobots_Overall = 0;
      float poses[Nb_blocks+20][2];
	
   /****************************************/
   /****************************************/
   /* Setup of files used for recording data and other initialization and setup */

   void CBoilerplateLoopFunctions::Init(TConfigurationNode& t_tree) {
	   
	srand(time(0));
	   
	/* Get output file name from XML tree */
   	GetNodeAttribute(t_tree, "output", m_strOutFile);
	std::time_t t = std::time(0);   // get time now
        std::tm* now = std::localtime(&t);
        std::cout << (now->tm_year + 1900) << '-' 
        << (now->tm_mon + 1) << '-'
        <<  now->tm_mday << '_'
	<< now->tm_hour << ":"
	<< now->tm_min << ":"
	<< now->tm_sec
        << "\n";
        auto tt = std::time(nullptr);
        auto tmt = *std::localtime(&tt);
	oss << std::put_time(&tmt, "%d-%m-%Y_%H-%M-%S");
	auto str = oss.str();
	str = "experiments/" + str;
	const int dir_err = mkdir(str.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (-1 == dir_err)
	{
	    printf("Error creating directory!n");
	    exit(1);
	}
	   
	m_strOutFile2 = str + "/boilerplate.argos";
	std::ifstream  src("boilerplate.argos", std::ios::binary);
    	std::ofstream  dst(m_strOutFile2.c_str(),   std::ios::binary);
    	dst << src.rdbuf();
	m_strOutFile = str + "/" + m_strOutFile;
   	std::cout << str << std::endl;

   	m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
  	if(m_cOutFile.fail()) {
      		THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
   	}

	c3=-1; // a counter for blocks
	int flagPos = 0; // for distributing blocks

   }

   /****************************************/
   /****************************************/
   /* terminating experiments at time step 50.000 */
	
   bool CBoilerplateLoopFunctions::IsExperimentFinished() {

	if (simcount==50000){ // simulation stops at time step 50.000
		return true;		
	}

	return false;
	
   }  

   /****************************************/
   /****************************************/
   /* main loop */
   void CBoilerplateLoopFunctions::PostStep() {

	int c=0; // a counter for all the objects in the environment (blocks, ground robots, and UAVs)
	simcount = simcount + 1; // simulation step is increased by one
	
	CSpace::TMapPerType& cFBMap = GetSpace().GetEntitiesByType("prototype");
	   
	Average_AllRobots_Local = 0; //average of all robots' local opinion at each step
	for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) {
	        CPrototypeEntity* cFB = any_cast<CPrototypeEntity*>(it->second); // Create a reference to the obstacles and the robots for easier coding
		
		//******************************************************************	
		/* static or dynamic setups based on boolean flag that is either 0 or non-zero
		if flag (i.e., flag position) is set to 0 the obstacles are distributed
		the flag is always set to 0 in the first time step
		the flag is always set to non-zero in the second time step
		in every repeated time period X with a manually set length, the flag has the possibility to be set to 0
		in dynamic setups, the flag is set to 0 at the start of a new time period X, and then set to non-zero in the next time step
		in static setups, the flag is never again set to 0 
		*/
		
		/* define whether this experiment setup is dyanmic or static */		
		if ((simcount)%10000 == 0){ // define the length of time period X
			flagpostion = 3; // define what the flag will be set to at the start of new time period X. this will be 0 in dynamic setups, non-zero in static
		}	
		if (flagpostion == 0  and cFB->HasComponent("leds.led[led_0]")){
			flagPos = 0;
			c3 = c3 + 1;			
			CLEDEntity& cLed = cFB->GetComponent<CLEDEntity>("leds.led[led_0]");
			while (flagPos != 1){ // generate a new obstacle at a random position in the (6m x 6m) arena with a buffer zone of (0.13m) from each border, and then check if it should be kept					
				posCounter = 0;
				float aa = ((float)rand() / RAND_MAX) * (2.87 + 2.87) - 2.87; // random x for new obstacle
				float bb = ((float)rand() / RAND_MAX) * (2.87 + 2.87) - 2.87; // random y for new obstacle
				
				if(c3 == 0){ // if it is the first obstacle placed in the arena 
					flagPos = 1;
					poses[c3][0] = aa;
					poses[c3][1] = bb;
					cFB->GetEmbodiedEntity().MoveTo({aa,bb,cFB->GetEmbodiedEntity().GetOriginAnchor().Position[2]}, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
				}
				else{  // else it is not the first obstacle, so check whether this new random obstacle should be kept
					int flagCheck = 0; // when flagCheck is 0, this indicates the new obstacle is safe
					for (int j=0;j<Nb_robots;j++){ // check that the x and y positions of the obstacle are at least 15 cm away from the x and y, respectively, of all of the ground robots. this needs to be checked because sometimes obstacles are placed after an experiment has already started, i.e. dynamic setup
						if (abs(aa - RobotCoordinate[j][0])<0.15 and abs(bb - RobotCoordinate[j][1])<0.15){
							flagCheck = 1; // when flagCheck is 1, this indicates the new obstacle is unsafe (i.e., did not pass the check)
							break;
						}
					}
					for (int i=0;i<c3;i++){ // check that the x and y positions of the obstacle are at least 20 cm away from the x and y, respectively, of all previously placed obstacles
						if ((abs(aa - poses[i][0])<0.2 and abs(bb - poses[i][1])<0.2) or flagCheck == 1){
							break;
						}
						else{posCounter = posCounter + 1;
						}
					}
					if (posCounter == c3){	// place the obstacle if it passes the two checks							
						flagPos = 1;
						poses[c3][0] = aa;
						poses[c3][1] = bb;	
						if (c3>50 and outFlag == 2){ // in the static scenario this "if" statement is not executed as outFlag is initially set to 1; in the dynamic scenario when outFlag is set to 1 the extra blocks are temporarily placed outside the arena (only 50 blocks are distributed in the arena)
							float xx = 3.6 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(12.9-3.6)));
							float  yy = ((float)rand() / RAND_MAX) * (12.9 + 12.9) - 12.9;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Position.SetX(xx);
							cFB->GetEmbodiedEntity().GetOriginAnchor().Position.SetY(yy);
							MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);

						}
						else{
							cFB->GetEmbodiedEntity().MoveTo({aa,bb,0.2}, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
						}
					}
				}
			}
			
		}
		
		//******************************************************************
		/* checks if a robot is close enough to an obstacle for the respective robot to see the respective obstacle */
		if (cFB->HasComponent("leds.led[led_0]")){ // if the object is a block
			
			if (evaporation[c] > 0){	
				evaporation[c] = evaporation[c] - 1;
			}

			CLEDEntity& cLed = cFB->GetComponent<CLEDEntity>("leds.led[led_0]");
			distance_1;
			colorFlag = 0;
			
			if (cLed.GetColor() != CColor::BLUE) { // if the block holds a pheromone source (i.e., if the block is blue)
			for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) { // for all robots
				CPrototypeEntity* cFB2 = any_cast<CPrototypeEntity*>(it->second);
				
				if (cFB2->HasComponent("tags")){ // if the object is a robot

					distance_1 = pow(pow((cFB2->GetEmbodiedEntity().GetOriginAnchor().Position[1]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]),2)+pow((cFB2->GetEmbodiedEntity().GetOriginAnchor().Position[0]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]),2),1/2.0);
					// distance_1 is the distance between a robot and the block
					
					if (distance_1 < 0.084 and distance_1 > 0 and cLed.GetColor() != CColor::PURPLE and cLed.GetColor() != CColor::BLUE){ // observable distance is : the center of the obstacle is less than 0.084 meters from the center of the robot  
						if (cLed.GetColor() == CColor::GREEN or cLed.GetColor() == CColor::RED){
							cLed.SetColor(CColor::BLUE);
						}
						else{
							cLed.SetColor(CColor::BLUE);
							evaporation[c] = 20000; //when a block is activated by a ground robot, it emmits pheromone for 20000 steps
							colorFlag = 1;
							break;
						}//end of else

					}
					
				}

			}
			}//end of if cLed.GetColor() != CColor::BLUE
			
                        /*new obstacles are set to "off" when they are placed*/
			if (simcount > 75000 and (simcount)%10000 == 0){ //this if statement is not executed in the static setups, but in case of dynamic setups, depending on the speed of change, we replace it with "if ((simcount)%10000 == 0){" or "if ((simcount)%10000 == 0){"
				if (cLed.GetColor() == CColor::BLUE){
					cLed.SetColor(CColor::BLACK);
				}
			}
			if (cLed.GetColor() != CColor::WHITE and colorFlag == 0 and evaporation[c] == 0 and cLed.GetColor() != CColor::PURPLE and cLed.GetColor() != CColor::BLACK){
				cLed.SetColor(CColor::BLACK); //set the obstacle to "off" when its corresponding evaporation counter becomes 0

			}

		 }	
		
		 //******************************************************************
		 if (c>(Nb_blocks+1)){  // i.e., if it is a ground robot
			
			int boxCount3 = 0; // a counter for the number of active blocks observed in one step by the ground robot, within a range of 50 cm from the center of the robot
			
			/* count number of active blocks within the range of 50 cm from the center of the ground robot */ 
			for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) { // for all objects
				CPrototypeEntity* cFB5 = any_cast<CPrototypeEntity*>(it->second);
				if (cFB5->HasComponent("leds.led[led_0]")){ // if the object is a block
					CLEDEntity& cLed2 = cFB5->GetComponent<CLEDEntity>("leds.led[led_0]");
					
					distance_2 = pow(pow((cFB5->GetEmbodiedEntity().GetOriginAnchor().Position[1]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]),2)+pow((cFB5->GetEmbodiedEntity().GetOriginAnchor().Position[0]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]),2),1/2.0); //distance between the block and the ground robot
					// distance_2 is the euclidian distance between the centerpoint of the robot and the centerpoint of the respective block
					
					/* define the distance at which a robot can observe an active block */
					if (distance_2 <= 0.5 and distance_2 > 0 and cLed2.GetColor() == CColor::BLUE){ // if the block is within 50 cm and holds a pheromone source (i.e., is blue)
						boxCount3 = boxCount3 + 1; // then increase the counter 						
					}
				}
			}
			double cumulative = 0; // summation of estimates
			ListOfBlocksDensities[c-(Nb_blocks+2)][simcount] = boxCount3/0.785; //the estimated block density in one step (the radius of observation is 0.5m, so the area of the circle of observation is 0.5*0.5*3.14 = 0.785)
			// when the steps are over 1000, then simply consider the last 1000 steps
			if (simcount>1000) { // for steps greater than 1000
				for (int i=simcount;i>(simcount-1000);i--){ // the summation of the block densities corresponing to the past 1000 steps
					cumulative = cumulative + ListOfBlocksDensities[c-(Nb_blocks+2)][i];
				}
			}
			// when the steps are less than 1000, then consider only the steps taken (the entries for untaken steps are 0), even though the sum of these steps is still divided by 1000
			if (simcount<=1000){ // for steps less than or equal to 1000
				for (int i=1000;i>0;i--){ // the sumation of the block densities corresponing to the past 1000 steps
					cumulative = cumulative + ListOfBlocksDensities[c-(Nb_blocks+2)][i];
				}	
			}
			// divide the sum of the considered steps by 1000
			opinion[c-(Nb_blocks+2)][simcount] = cumulative/1000; // calculate the opinion as the average of block densities corresponing to the past 1000 steps- the whole process, from line 253 to 268, is equivalent to calculating the summation of observed blocks per step during the past 1000 steps, and deviding the result by the total area of observation which is 1000*"area of observation in one step" (i.e., 1000*0.785 m^2)
			
			m_cOutFile << floor((boxCount3/0.785 * pow(10, 3) + 0.5)) / pow(10, 3)  << "\t" ; // record the block density of the current step
			m_cOutFile << floor((opinion[c-(Nb_blocks+2)][simcount] * pow(10, 3) + 0.5)) / pow(10, 3) << "\t" ; // record the local opinion of the current step

			Average_AllRobots_Local = Average_AllRobots_Local + opinion[c-(Nb_blocks+2)][simcount]; // the average opinion of all robots
	   
			 
			//******************************************************************
			/* How ground robots react to the arena boundary */
			RobotCoordinate[c-(Nb_blocks+2)][0] = cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]; // the ground robot's x coordinate 
			RobotCoordinate[c-(Nb_blocks+2)][1] = cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]; // the ground robot's y coordinate 

			argos::CQuaternion orient = cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation;
			CRadians zAngle, yAngle, xAngle;
			cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(zAngle, yAngle, xAngle);
			float z = orient.GetZ();
				
			if (cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]<-3){ //a ground robot turns to a direction facing away from southern border when it reaches it
				v = rand() % 2;
				if (v == 0) {
					v1 = rand() % 180;
					cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(v1,0,0, 180);
				}
				else if (v == 1) {
					v1 = rand() % 180;
					cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(180,0,0, v1);
				} 
				MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
			}
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]<-3){ // a ground robot turns to a direction facing away from western border when it reaches it
				v = rand() % 2;
				if (v == 0) {
					v1 = rand() % 180;
					cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(180,0,0, v1);
				}
				else if (v == 1) {
					v1 = rand() % 180;
					cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(180,0,0, -1*v1);
				} 

				MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
			}
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]>3){ // a ground robot turns to a direction facing away from northern border when it reaches it
				v = rand() % 2;
				if (v == 0) {
					v1 = rand() % 180;
					cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(v1,0,0, -180);
				}
				else if (v == 1) {
					v1 = rand() % 180;
					cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(180,0,0, -1*v1);
				} 

				MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
			}
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]>3){ // a ground robot turns to a direction facing away from eastern border when it reaches it
				v = rand() % 2;

				if (v == 0) {
					v1 = rand() % 180;
					cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(v1,0,0, 180);
				}
				else if (v == 1) {
					v1 = rand() % 180;
					cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(-1*v1,0,0, 180);
				} 

				MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
			}		
			
		}

		c=c+1;
		//******************************************************************
		/* reinitializing counters and flags to prepare for the next loop */
		if (c3 == Nb_blocks) {
			flagpostion = 1;
			c3 = -1;
			if (outFlag == 0){
				outFlag = 1;
			}
			else{
				outFlag = 0;
			}
		}
	}//end of the for loop of robot space


	//******************************************************************
	m_cOutFile << floor((Average_AllRobots_Local/Nb_robots * pow(10, 3) + 0.5)) / pow(10, 3) << "\t"; // record the average of local densities for all robots calculated in the current step
	m_cOutFile << GetSpace().GetSimulationClock() << std::endl;			
	
   }
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CBoilerplateLoopFunctions, "boilerplate_loop_functions");

}
