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
#include<cmath>
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
      const int Nb_robots = 8; // number of robots
      const int Nb_blocks = 100; // number of blocks
      float RobotCollision[Nb_robots][2]; // robots' postion (x and y coordinates) is stored in this array: for 8 ground robots
      int evapolation[Nb_blocks+1] = {};
      float poses[Nb_blocks+20][2];	
      int boxCounter = 0; // for counting the number of observed blocks
      int outFlag = 0; // for dynamic setup
      float ListOfBlocksCounted[50500]; // the total numbr of blocks observed from the beginning of the run till the current step
      float ListOfWidth[50500]; // indicates the width of MNS at each step (the width is equal to 1 m when the MNS moves forward or backward; the width is equal to 0.25 m when the MNS shifts to left or right) 
      float OverallAverage_1000[50500]; // an array to store overall opinion at each step
      double opinion_1500[50500]; // an array to store the opinion corresponding to the past 1500 steps at each step
      double opinion_1000[50500]; // an array to store the opinion corresponding to the past 1000 steps at each step
      double opinion_1200[50500]; // an array to store the opinion corresponding to the past 1200 steps at each step
	
   /****************************************/
   /****************************************/

   void CBoilerplateLoopFunctions::Init(TConfigurationNode& t_tree) {

	srand(time(0));
	
	/* Get output file name from XML tree */
   	GetNodeAttribute(t_tree, "output", m_strOutFile);
	//m_strOutFile = "experiments/" + m_strOutFile;
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
	LOG << "File: " << str << std::endl;
	LOG << "tree" << m_strOutFile << std::endl;
  	/* Open the file for text writing */
   	m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
  	if(m_cOutFile.fail()) {
      		THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
   	}
	
	c3=-1; // a counter for blocks
	int flagPos = 0;

   }

   /****************************************/
   /****************************************/
   bool CBoilerplateLoopFunctions::IsExperimentFinished() {
	
	if (simcount == 50320){ // simulation stops at time step 50.000; the first 320 steps are for forming the MNS and are not counted
		return true;		
	}

	return false;
	
   }  

   /****************************************/
   /****************************************/

   void CBoilerplateLoopFunctions::PostStep() {
	
	int c=0; // a counter for all the objects in the environment (blocks, ground robots, and UAVs)
	simcount = simcount + 1; // simulation step is increased by one
		
	CSpace::TMapPerType& cFBMap = GetSpace().GetEntitiesByType("prototype");
	
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
		if ((simcount-320)%1000 == 0){ // define the length of time period X
			flagpostion = 3; // define what the flag will be set to at the start of new time period X. this will be 0 in dynamic setups, non-zero in static
		}
			
		if (flagpostion == 0  and cFB->HasComponent("leds.led[led_0]")){
			flagPos = 0;
			c3 = c3 + 1;
			CLEDEntity& cLed = cFB->GetComponent<CLEDEntity>("leds.led[led_0]");
			while (flagPos != 1){ // generate a new obstacle at a random position and then check if it should be kept	
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
						if (abs(aa - RobotCollision[j][0])<0.15 and abs(bb - RobotCollision[j][1])<0.15){
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
						
						// for dynamic environment: we can put some boxes outside the arena
	
						if (c3>50 and outFlag == 2){ // in the static scenario this "if" statement is not executed as outFlag is initially set to 0; in the dynamic scenario when outFlag is set to 1 the extra blocks are temporarily placed outside the arena (only 50 blocks are distributed in the arena)
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
		if (cFB->HasComponent("leds.led[led_0]")){ //if the object is a block			
			if (evapolation[c] > 0){	
				evapolation[c] = evapolation[c] - 1;
			}
			
			CLEDEntity& cLed = cFB->GetComponent<CLEDEntity>("leds.led[led_0]");

			distance;
			colorFlag = 0;
			int OutofView = 1; //a flag to ignore the blocks sensed by either the righmost or leftmost ground robot in the formation when they are out of view of the line of ground robots  
			if (cLed.GetColor() != CColor::BLUE) {
				for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) {
					CPrototypeEntity* cFB5 = any_cast<CPrototypeEntity*>(it->second);
					if (cFB5->HasComponent("tags")){
						if (cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1] < cFB5->GetEmbodiedEntity().GetOriginAnchor().Position[1]){
							OutofView = 0; // the flag becomes 0 when an observed block is located between the rightmost and leftmost ground robot 
						}
					}
				}

				for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) {
					CPrototypeEntity* cFB2 = any_cast<CPrototypeEntity*>(it->second);

					if (cFB2->HasComponent("tags")){

						distance = pow(pow((cFB2->GetEmbodiedEntity().GetOriginAnchor().Position[1]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]),2)+pow((cFB2->GetEmbodiedEntity().GetOriginAnchor().Position[0]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]),2),1/2.0);

						/*except for the first lane (i.e., when simcount<2500 or (simcount>25100 and simcount<29200)),
						when distance between a ground robot and a block is less than 0.084 m and the block 
						is located between the leftmost and rightmost ground robots in the formation, the block
						is activated for 300 steps and couted by the MNS*/
						if ((simcount<2500 or (simcount>25100 and simcount<29200) or OutofView == 0) and distance < 0.084 and distance > 0 and cLed.GetColor() != CColor::PURPLE and cLed.GetColor() != CColor::BLUE and simcount>320){  // observable distance is : the center of the obstacle is less than 0.084 meters from the center of the robot
							cLed.SetColor(CColor::BLUE);
							if (simcount>320){
								++boxCounter; // for counting the number of observed blocks
							}
							evapolation[c] = 300; //when an obstacle is activated by a ground robot, it is set to "on" for 300 steps
							colorFlag = 1;
							break;
						}

					}

				}
			}//end of if cLed.GetColor() != CColor::BLUE
			
			/*new obstacles are set to "off" when they are placed*/
			if (simcount>70320 && (simcount-320)%1000 == 0){ //this if statement is not executed in the static setups, but in case of dynamic setups, depending on the speed of change, we replace it with "if ((simcount-320)%1000 == 0){" or "if ((simcount-320)%1000 == 0){" 
				if (cLed.GetColor() == CColor::BLUE){
					cLed.SetColor(CColor::BLACK);
				}
			}
			if (cLed.GetColor() != CColor::WHITE and colorFlag == 0 and evapolation[c] == 0 and cLed.GetColor() != CColor::PURPLE and cLed.GetColor() != CColor::BLACK){
				cLed.SetColor(CColor::BLACK); //set the obstacle to "off" when its corresponding evaporation counter becomes 0
			}
			
		}		

		//******************************************************************
		if (c>(Nb_blocks+1)){  // i.e., if it is a ground robot 
			RobotCollision[c-(Nb_blocks+2)][0] = cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]; // the ground robot's x coordinate
			RobotCollision[c-(Nb_blocks+2)][1] = cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]; // the ground robot's y coordinate
			
                        /* Lines 256 to 314 simulates ground robots reaction to the arena boundary if the MNS passes a boundary by 1 meter*/
			/*As the MNS never goes out of the boundaries in this project, lines 256 to 314 are useless can be removed*/
			argos::CQuaternion orient = cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation;
			CRadians zAngle, yAngle, xAngle;
			cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(zAngle, yAngle, xAngle);
			float z = orient.GetZ();
				
			if (cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]<-4){
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
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]<-4){
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
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]>4){
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
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]>4){
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
	///////////// Calculating opinion, based on block counted 
	//******************************************************************
	std::ifstream infile5("density_value.csv"); // during the time that MNS is next to a boundary, value 1 is written in density_value.csv by brain; otherwise value 0 is written
	float fi5;

	ListOfBlocksCounted[simcount-1] = boxCounter; // the total numbr of blocks observed from the beginning of the run

	int NumberOfBlocks_MovingWindow_1500 = 0; // to count the number of blocks observed during the past 1500 steps
	int NumberOfBlocks_MovingWindow_1000 = 0; // to count the number of blocks observed during the past 1000 steps
	int NumberOfBlocks_MovingWindow_1200 = 0; // to count the number of blocks observed during the past 1200 steps

	infile5 >> fi5;
	double Sum_of_width_1000 = 0; // the total width of observation area during the past 1000 steps
	double Sum_of_width_1200 = 0; // the total width of observation area during the past 1000 steps
	double Sum_of_width_1500 = 0; // the total width of observation area during the past 1000 steps
	if (simcount > 1000 &&  fi5== 1){
		ListOfWidth[simcount-1] = 0.25; // when MNS reaches a boundary, it shifts to left or right to sweep a neighboring lane; in such a case its width is equal to 0.25 m
	}
	else{
		ListOfWidth[simcount-1] = 1; // when MNS moves forward or backward its width is equal to 1 m
	}
	if (simcount > 1320){ // for steps greater than 1000
		for (int k = (simcount-1); k>(simcount-1001); k--){ // the summation of width of observation area during the past 1000 steps
			Sum_of_width_1000 = Sum_of_width_1000 + ListOfWidth[k];
		}
		NumberOfBlocks_MovingWindow_1000 = ListOfBlocksCounted[simcount-1] - ListOfBlocksCounted[simcount-1001]; // the number of blocks observed during the past 1000 steps
		opinion_1000[simcount-1] = NumberOfBlocks_MovingWindow_1000/(Sum_of_width_1000*0.002985); // the opinion corresponding to the past 1000 steps
	}
	if (simcount > 1520){ // for steps greater than 1200
		for (int k = (simcount-1); k>(simcount-1201); k--){ // the summation of width of observation area during the past 1200 steps
			Sum_of_width_1200 = Sum_of_width_1200 + ListOfWidth[k];
		}
		NumberOfBlocks_MovingWindow_1200 = ListOfBlocksCounted[simcount-1] - ListOfBlocksCounted[simcount-1201]; // the number of blocks observed during the past 1200 steps
		opinion_1200[simcount-1] = NumberOfBlocks_MovingWindow_1200/(Sum_of_width_1200*0.002985); // the opinion corresponding to the past 1200 steps 
	}
	if (simcount > 1820){ // for steps greater than 1500
		for (int k = (simcount-1); k>(simcount-1501); k--){ // the summation of width of observation area during the past 1500 steps
			Sum_of_width_1500 = Sum_of_width_1500 + ListOfWidth[k];
		}
		NumberOfBlocks_MovingWindow_1500 = ListOfBlocksCounted[simcount-1] - ListOfBlocksCounted[simcount-1501]; // the number of blocks observed during the past 1500 steps
		opinion_1500[simcount-1] = NumberOfBlocks_MovingWindow_1500/(Sum_of_width_1500*0.002985); // the opinion corresponding to the past 1500 steps
	}
	if (simcount <= 1320){  // for steps less than or equal to 1000; during this time the MNS moves forward
		NumberOfBlocks_MovingWindow_1000 = ListOfBlocksCounted[simcount-1] - ListOfBlocksCounted[0]; // the number of blocks observed during the past 1000 steps
		opinion_1000[simcount-1] = NumberOfBlocks_MovingWindow_1000/(1000*0.002985*1); // the opinion corresponding to the past 1000 steps
	}
	if (simcount <= 1520){  // for steps less than or equal to 1200; during this time the MNS moves forward 
		NumberOfBlocks_MovingWindow_1200 = ListOfBlocksCounted[simcount-1] - ListOfBlocksCounted[0]; // the number of blocks observed during the past 1200 steps
		opinion_1200[simcount-1] = NumberOfBlocks_MovingWindow_1200/(1200*0.002985*1); // the opinion corresponding to the past 1200 steps
	}
	if (simcount <= 1820){  // for steps less than or equal to 1500; during this time the MNS moves forward 
		NumberOfBlocks_MovingWindow_1500 = ListOfBlocksCounted[simcount-1] - ListOfBlocksCounted[0]; // the number of blocks observed during the past 1500 steps
		opinion_1500[simcount-1] = NumberOfBlocks_MovingWindow_1500/(1500*0.002985*1); // the opinion corresponding to the past 1500 steps
	}

	double OverallSum_1000 = 0; // to calculate the overall opinion (not used in this project)
	for (int i=0;i<=simcount-1;i++){ // the summation of all opinions (relevant to each 1000-steps) from beginning of the run till the current step
		OverallSum_1000 = OverallSum_1000 + opinion_1000[i];			
	}
	OverallAverage_1000[simcount-1] = OverallSum_1000/simcount; // the overall opinion (recorded but not used in this project)
	if (simcount > 320){
		m_cOutFile << GetSpace().GetSimulationClock()-320 << "\t" // time step
		<< boxCounter << "\t" // the total number of observed blocks
		<< floor((opinion_1000[simcount-1] * pow(10, 3) + 0.5)) / pow(10, 3) << "\t" // the opinion corresponding to the past 1000 steps
		<< floor((opinion_1200[simcount-1] * pow(10, 3) + 0.5)) / pow(10, 3) << "\t" // the opinion corresponding to the past 1200 steps
		<< floor((opinion_1500[simcount-1] * pow(10, 3) + 0.5)) / pow(10, 3) << "\t" // the opinion corresponding to the past 1500 steps
		<< floor((OverallAverage_1000[simcount-1] * pow(10, 3) + 0.5)) / pow(10, 3) << "\t" // the overall opinion

		<< std::endl;
	}


   }
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CBoilerplateLoopFunctions, "boilerplate_loop_functions");

}
