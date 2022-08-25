# id of type, corresponded to object base
int32 type_id                               

# name of type, corresponded to object base
string type_name                           

# confidence coefficient of current result
float64 score                             

# additional info represended as dictionary
extended_object_detection/ExtractedInfo extracted_info

# object position and geometry

## transform to object center
geometry_msgs/Transform transform           

## object frames on image and 3d coordinates
extended_object_detection/Rect rect         

# information about tracking if it is enabled
extended_object_detection/Track track       




