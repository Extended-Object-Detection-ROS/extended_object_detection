# object info
int32 type_id                               # id of type, corresponded to object base
string type_name                            # name of type, corresponded to object base
float64 score                               # confidence coefficient of current result
extended_object_detection/ExtractedInfo[] extracted_info

# object position and geometry
geometry_msgs/Transform transform           # transform to object center
extended_object_detection/Rect rect         # object frames on image and 3d coordinates
extended_object_detection/Contour contour   # contour image and 3d info

# tracking
extended_object_detection/Track track       # information about tracking if it is enabled




