#ifndef BOILERPLATE_LOOP_FUNCTIONS_H
#define BOILERPLATE_LOOP_FUNCTIONS_H
#include<time.h>
#include <sys/stat.h> 
#include <sys/types.h>
#include <exception>
#include <experimental/filesystem>
#include <iostream>
#include <functional>
#include <fstream>


namespace argos {
   class CPrototypeEntity;
}

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_radios_actuator.h>
#include <map>

namespace argos {
namespace fs = std::experimental::filesystem;
   class CBoilerplateLoopFunctions : public CLoopFunctions {

   public:

      virtual void Init(TConfigurationNode& t_tree);
      
      virtual bool IsExperimentFinished();
      //virtual void Reset();

      //virtual void Destroy();

      //virtual void PreStep();

      virtual void PostStep();

   //The path of the output file.
   std::string m_strOutFile;
   std::string m_strOutFile2;

   //The stream associated to the output file.
   std::ofstream m_cOutFile;
   std::ifstream infile5;

   std::ostringstream oss;

   private:
      int v1;
      int v;
      float a;
      float b;
      int c2=0;
      float distance;
      int colorFlag;
      int c3 = -1;
      int posCounter;
      int flagpostion = 0;
      int flagPos = 0;
      float enVec[48][48][7];
      int simcount = 0;
      float currentPercentage = 0;
      int zeroCounter = 0;
      CPrototypeEntity* cFB;
   };
}

#endif

