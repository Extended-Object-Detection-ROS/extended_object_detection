#ifndef RELATION_SHIP_H
#define RELATION_SHIP_H

//#include "SimpleObject.h"
#include "ExtendedObjectInfo.h"
#include "tinyxml.h"

namespace eod{
        
    enum RelationTypes{
        UNK_R = 0,
        IM_RANGE_R,
        LOG_AND_R,
        LOG_NOT_R,
        LOG_OR_R,
        TD_RANGE_R,
        SPACE_IN_R,
        SPACE_OUT_R,
        SPACE_UP_R,
        SPACE_DOWN_R,
        SPACE_LEFT_R,
        SPACE_RIGHT_R,
        SIZE_SAME_R,
        SIZE_BIGGER_R,
        SIZE_SMALLER_R,
        SIZE_PERCENT_R,
    };
    
    RelationTypes getRelationTypeFromName(std::string name);
    
    class RelationShip{
    public:
      std::string Name;              
      int ID;

      /// <summary>
      /// Default constructor
      /// </summary>		           
      RelationShip();
      
      /// <summary>
      /// Prints information about relation.
      /// </summary>		   
      //virtual void printInfo() = 0;
      
      /// <summary>
      /// Checks if objects are satisfied to relation
      /// </summary>		 
      /// <param name="A">Pointer to first object</param>
      /// <param name="B">Pointer to second object</param>
      /// <returns>True or false</returns>	      
      virtual bool checkRelation(const cv::Mat& image, ExtendedObjectInfo* A, ExtendedObjectInfo* B) = 0;
      
      //virtual bool readFromXML(TiXmlElement* relation_tag) = 0;
      
      bool setName(const char*);
	
    private:
        
    protected:
        int Type;        
        bool inited;
		
    };
}

#endif //RELATION_SHIP_H
