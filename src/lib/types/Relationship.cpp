#include "Relationship.h" 

using namespace std;

namespace eod{
    
    RelationTypes getRelationTypeFromName(string name){
        transform(name.begin(), name.end(), name.begin(),[](unsigned char c){ return tolower(c); });
        
        if( name == "imagerange" )
            return IM_RANGE_R;            
        if( name == "logicand" )
            return LOG_AND_R;
        if( name == "logicnot" )
            return LOG_NOT_R;
        if( name == "logicor" )
            return LOG_OR_R;
        if( name == "3drange" )
            return TD_RANGE_R;
        if( name == "spacein" )
            return SPACE_IN_R;
        if( name == "spaceout" )
            return SPACE_OUT_R;
        if( name == "spaceup" )
            return SPACE_UP_R;
        if( name == "spacedown" )
            return SPACE_DOWN_R;
        if( name == "spaceleft" )
            return SPACE_LEFT_R;
        if( name == "spaceright" )
            return SPACE_RIGHT_R;
        
        printf("Unknown relation type %s!",name.c_str());
        return UNK_R;
    }
    
    RelationShip::RelationShip(){
        Type = UNK_R;
        inited = false;
    }        
    
    bool RelationShip::setName(const char* name){
        if( !name )
            return false;
        Name = string(name);      
        return true;
    }
}
