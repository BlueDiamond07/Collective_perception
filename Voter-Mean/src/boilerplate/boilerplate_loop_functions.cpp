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
      const int Nb_robots = 8;
      const int Nb_blocks = 100;	
      float RobotCoordinate[Nb_robots][2]; // for Nb_robots ground robots
      int disCounter[Nb_robots+Nb_blocks+2] = {};
      int boxCount[Nb_robots] = {};
      int evaporation[Nb_blocks+1] = {};
      int id;
      int outFlag = 1; // for dynamic scenario
      float poses[Nb_blocks+20][2];
	
   /****************************************/
   /****************************************/

   /* creating data to save to csv */
	
   void CBoilerplateLoopFunctions::Init(TConfigurationNode& t_tree) {

	srand(time(0));
	   
	m_strOutFile = "Robots-Memories/Results/boilerplate.argos";
	std::ifstream  src("boilerplate.argos", std::ios::binary);
    	std::ofstream  dst(m_strOutFile.c_str(),   std::ios::binary);
    	dst << src.rdbuf();

	c3=-1; // a counter for blocks
	int flagPos = 0;

   }

	
   /****************************************/
   /****************************************/
   /* termination of experiment */
	
   bool CBoilerplateLoopFunctions::IsExperimentFinished() {

	if (simcount==50000){ // simulation stops at time step 50000
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
		
		//******************************************************************
		/* place obstacles in the arena every time the flag is set to 0 */
		if (flagpostion == 0  and cFB->HasComponent("leds.led[led_0]")){   
			flagPos = 0;
			c3 = c3 + 1;			
			CLEDEntity& cLed = cFB->GetComponent<CLEDEntity>("leds.led[led_0]");
			while (flagPos != 1){	// generate a new obstacle at a random position and then check if it should be kept			
				posCounter = 0;
				float aa = ((float)rand() / RAND_MAX) * (2.87 + 2.87) - 2.87; // random x for new obstacle
				float bb = ((float)rand() / RAND_MAX) * (2.87 + 2.87) - 2.87; // random y for new obstacle
				
				if(c3 == 0){  // if it is the first obstacle placed in the arena
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
					for (int i=0;i<c3;i++){  // check that the x and y positions of the obstacle are at least 20 cm away from the x and y, respectively, of all previously placed obstacles						
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
							float yy = ((float)rand() / RAND_MAX) * (12.9 + 12.9) - 12.9;
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
		if (cFB->HasComponent("leds.led[led_0]")){
			
			if (evaporation[c] > 0){	
				evaporation[c] = evaporation[c] - 1;
			}

			CLEDEntity& cLed = cFB->GetComponent<CLEDEntity>("leds.led[led_0]");
			distance_1;  // the distance at which a robot can see an obstacle
			colorFlag = 0;
			
			if (cLed.GetColor() != CColor::BLUE) {
				for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) {
					CPrototypeEntity* cFB2 = any_cast<CPrototypeEntity*>(it->second);

					if (cFB2->HasComponent("tags")){ 

						distance_1 = pow(pow((cFB2->GetEmbodiedEntity().GetOriginAnchor().Position[1]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]),2)+pow((cFB2->GetEmbodiedEntity().GetOriginAnchor().Position[0]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]),2),1/2.0);

						/* define the distance at which a robot can see an obstacle
						a robot can see a block if the center of the block is within 0.084 m of the center of the robot, in any direction (i.e., all proximity sensors are used)
						specifically, a robot can see a block if the edge of the robot is 2.4-3.4 cm away from the edge of the block
						if the block is oriented exactly perpendicular to the normal vector of the robot's edge, then the block edge is 3.4 cm away from the robot edge when detected
						if the block is oriented exactly at a 45 degree angle to the normal vector of the robot's edge (i.e., the corner is facing the robot), then the block edge is 2.4 cm away from the robot edge when detected 
						*/
						if (distance_1 < 0.084 and distance_1 > 0 and cLed.GetColor() != CColor::PURPLE and cLed.GetColor() != CColor::BLUE){  // observable distance is : the center of the obstacle is less than 0.084 meters from the center of the robot
							if (cLed.GetColor() == CColor::GREEN or cLed.GetColor() == CColor::RED){
								cLed.SetColor(CColor::BLUE);
							}
							else{
								cLed.SetColor(CColor::BLUE);
								//++boxCounter;
								evaporation[c] = 1; //the obstacle is set to "on" for 1 step
								colorFlag = 1;
								break;
							}//end of else

						}
					}

				}
			}//end of if cLed.GetColor() != CColor::BLUE

			/* minor implementation detail that is only relevant to the pheromone approach. new obstacles are set to "off" when they are placed*/
			if (simcount > 75000 and (simcount)%10000 == 0){ //this if statement is not executed in the static setups, but in case of dynamic setups, depending on the speed of change, we replace it with "if ((simcount)%10000 == 0){" or "if ((simcount)%10000 == 0){"
				if (cLed.GetColor() == CColor::BLUE){
					cLed.SetColor(CColor::BLACK);
				}
			}
			
			if (cLed.GetColor() != CColor::WHITE and colorFlag == 0 and evaporation[c] == 0 and cLed.GetColor() != CColor::PURPLE and cLed.GetColor() != CColor::BLACK){ // only for pheromone
				cLed.SetColor(CColor::BLACK); //set the obstacle to "off" when its corresponding evaporation counter becomes 0

			}

		}		

		//******************************************************************
		/* in the non-pheromone setups, when a robot sees and records an obstacle, the respective robot cannot record the respective obstacle again for the next 40 time steps */
		 if (c>(Nb_blocks+1)){ // i.e., if it is a ground robot
			if(disCounter[c-(Nb_blocks+2)]==0){
					
				for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) { 
					CPrototypeEntity* cFB4 = any_cast<CPrototypeEntity*>(it->second);
					if (cFB4->HasComponent("leds.led[led_0]")){
						distance_2 = pow(pow((cFB4->GetEmbodiedEntity().GetOriginAnchor().Position[1]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]),2)+pow((cFB4->GetEmbodiedEntity().GetOriginAnchor().Position[0]-cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]),2),1/2.0);
						
						/* a robot records an obstacle if it is in the observable distance and it has not been recorded already in the last 40 time steps */
						if (distance_2 < 0.084 and distance_2 > 0 and disCounter[c-(Nb_blocks+2)]==0){ //for fault tolerance: (cFB->GetId() != "vehicle0" and cFB->GetId() != "vehicle1" and cFB->GetId() != "vehicle2" and cFB->GetId() != "vehicle3") and 
							disCounter[c-(Nb_blocks+2)] = 40;
							
							/* finds the robot that is currently being checked */
							if (cFB->GetId() == "vehicle0"){boxCount[0] = boxCount[0] + 1; id = 0;} 
							else if (cFB->GetId() == "vehicle1"){boxCount[1] = boxCount[1] + 1;id = 1;}
							else if(cFB->GetId() == "vehicle2"){boxCount[2] = boxCount[2] + 1;id = 2;}
							else if(cFB->GetId() == "vehicle3"){boxCount[3] = boxCount[3] + 1;id = 3;}
							else if(cFB->GetId() == "vehicle4"){boxCount[4] = boxCount[4] + 1;id = 4;}
							else if(cFB->GetId() == "vehicle5"){boxCount[5] = boxCount[5] + 1;id = 5;}
							else if(cFB->GetId() == "vehicle6"){boxCount[6] = boxCount[6] + 1;id = 6;}
							else if(cFB->GetId() == "vehicle7"){boxCount[7] = boxCount[7] + 1;id = 7;}
							else if(cFB->GetId() == "vehicle8"){boxCount[8] = boxCount[8] + 1;id = 8;}
							else if(cFB->GetId() == "vehicle9"){boxCount[9] = boxCount[9] + 1;id = 9;}
							else if(cFB->GetId() == "vehicle10"){boxCount[10] = boxCount[10] + 1;id = 10;}
							else if(cFB->GetId() == "vehicle11"){boxCount[11] = boxCount[11] + 1;id = 11;}

							auto strR = "Robots-Memories/Others/" + cFB->GetId() + ".csv";
			
							m_cOutFile.open(strR.c_str(), std::ofstream::out | std::ofstream::trunc);
		
							m_cOutFile << boxCount[id] 
								 
								    << std::endl;
							m_cOutFile.close();
							
						}
							
					}
				}
			}
			//******************************************************************
			/* reduces the counter that shows the remaining time unitl the respective robot is allowed to record the respective obstacle */ 
			else if(disCounter[c-(Nb_blocks+2)]>0){
				disCounter[c-(Nb_blocks+2)] = disCounter[c-(Nb_blocks+2)] - 1;
			}
									 
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
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]<-3){ //a ground robot turns to a direction facing away from western border when it reaches it
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
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]>3){ //a ground robot turns to a direction facing away from northern border when it reaches it
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
			else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]>3){ //a ground robot turns to a direction facing away from eastern border when it reaches it
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
	} //end of the for loop of robot space
	
   }
   //******************************************************************

   REGISTER_LOOP_FUNCTIONS(CBoilerplateLoopFunctions, "boilerplate_loop_functions");

}
